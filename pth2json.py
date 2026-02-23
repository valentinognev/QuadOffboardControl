#!/usr/bin/env python3
"""
Convert PyTorch .pth files to JSON format for C++ consumption.
Stores tensor data along with shape and dtype information.
"""

import torch
import json
import sys
import os
import argparse
import base64
import numpy as np
from pathlib import Path


def tensor_to_dict(tensor):
    """
    Convert a PyTorch tensor to a dictionary with data, shape, and dtype.
    Data is stored as base64-encoded bytes for efficient storage.
    This format is easier to parse from C++.
    """
    # Ensure tensor is on CPU
    if tensor.device.type != 'cpu':
        tensor = tensor.cpu()
    
    # Convert tensor to numpy array and get raw bytes
    np_array = tensor.detach().numpy()
    # Ensure contiguous array for proper byte representation
    if not np_array.flags['C_CONTIGUOUS']:
        np_array = np.ascontiguousarray(np_array)
    
    # Get raw bytes and encode to base64
    raw_bytes = np_array.tobytes()
    base64_data = base64.b64encode(raw_bytes).decode('utf-8')
    
    return {
        'data': base64_data,
        'shape': list(tensor.shape),
        'dtype': str(tensor.dtype),
        'numel': int(tensor.numel()),  # Total number of elements
        'encoding': 'base64'  # Indicate the encoding format
    }


def make_json_serializable(obj):
    """
    Recursively convert an object to JSON-serializable format.
    Handles tensors, numpy arrays, and other non-serializable types.
    """
    if isinstance(obj, torch.Tensor):
        return tensor_to_dict(obj)
    elif hasattr(obj, 'item'):  # Handle numpy scalars and similar
        return obj.item()
    elif isinstance(obj, (dict,)):
        return {k: make_json_serializable(v) for k, v in obj.items()}
    elif isinstance(obj, (list, tuple)):
        return [make_json_serializable(item) for item in obj]
    elif isinstance(obj, (int, float, str, bool, type(None))):
        return obj
    else:
        # For other types, try to convert to string representation
        return str(obj)


def pth_to_json(pth_path, json_path=None):
    """
    Convert a PyTorch .pth file to JSON format.
    
    Args:
        pth_path: Path to the .pth file
        json_path: Optional output path. If None, uses pth_path with .json extension
    """
    # Validate input file
    if not os.path.exists(pth_path):
        raise FileNotFoundError(f"PyTorch file not found: {pth_path}")
    
    # Determine output path
    if json_path is None:
        json_path = str(Path(pth_path).with_suffix('.json'))
    
    print(f"Loading PyTorch model from: {pth_path}")
    
    # Load state dict
    # Handle PyTorch 2.6+ weights_only=True default
    # Try with weights_only=True first, fall back to weights_only=False if needed
    checkpoint = None
    try:
        # First attempt: try with weights_only=True (safer, default in PyTorch 2.6+)
        checkpoint = torch.load(pth_path, map_location='cpu', weights_only=True)
    except Exception as e:
        error_msg = str(e)
        # If it fails due to weights_only restrictions, try with weights_only=False
        if 'weights_only' in error_msg or 'WeightsUnpickler' in error_msg or 'allowed global' in error_msg.lower():
            print("Note: Using weights_only=False to load checkpoint (may contain numpy scalars or other objects)")
            try:
                checkpoint = torch.load(pth_path, map_location='cpu', weights_only=False)
            except Exception as e2:
                raise RuntimeError(f"Failed to load PyTorch file even with weights_only=False: {e2}")
        else:
            # Re-raise if it's a different error
            raise RuntimeError(f"Failed to load PyTorch file: {e}")
    
    if checkpoint is None:
        raise RuntimeError("Failed to load PyTorch file: checkpoint is None")
    
    # Handle checkpoints that are wrapped in "model" key (common in Sample Factory)
    if isinstance(checkpoint, dict) and 'model' in checkpoint:
        state_dict = checkpoint['model']
    else:
        state_dict = checkpoint
    
    # Convert tensors to dictionaries with metadata
    weights_dict = {}
    total_params = 0
    
    for key, value in state_dict.items():
        if isinstance(value, torch.Tensor):
            weights_dict[key] = tensor_to_dict(value)
            total_params += value.numel()
        else:
            # Handle non-tensor values (e.g., scalars, strings, nested structures)
            try:
                # Try to make it JSON serializable
                serialized_value = make_json_serializable(value)
                weights_dict[key] = {
                    'data': serialized_value,
                    'type': type(value).__name__
                }
                print(f"Warning: Non-tensor value found for key '{key}': {type(value)}")
            except Exception as e:
                # If serialization fails, store as string representation
                weights_dict[key] = {
                    'data': str(value),
                    'type': type(value).__name__,
                    'error': f"Could not serialize: {e}"
                }
                print(f"Warning: Could not serialize value for key '{key}': {e}")
    
    # Create output dictionary with metadata
    # Ensure all values are JSON serializable
    # Convert to absolute path for consistency
    pth_absolute_path = os.path.abspath(pth_path)
    
    output = {
        'original_pth_file': pth_absolute_path,
        'metadata': {
            'source_file': str(pth_path),
            'num_layers': int(len(weights_dict)),
            'total_parameters': int(total_params)
        },
        'weights': make_json_serializable(weights_dict)
    }
    
    # Save to JSON
    print(f"Saving JSON to: {json_path}")
    try:
        with open(json_path, 'w') as f:
            json.dump(output, f, indent=2)
        print(f"Successfully converted {len(weights_dict)} tensors ({total_params:,} parameters)")
    except Exception as e:
        raise RuntimeError(f"Failed to write JSON file: {e}")
    
    return json_path


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        usage='usage: pth2json.py [-h] --pth=pthfilepath [json_path]',
        description='Convert PyTorch .pth files to JSON format for C++ consumption.\n'
                    'Stores tensor data along with shape and dtype information as base64-encoded bytes.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='Examples:\n'
               '  python pth2json.py --pth=model.pth\n'
               '  python pth2json.py --pth=model.pth output.json\n'
               '  python pth2json.py --help  # Show this help message',
        add_help=True
    )
    parser.add_argument(
        '--pth',
        dest='pth_path',
        type=str,
        required=True,
        metavar='pthfilepath',
        help='Path to the input .pth file'
    )
    parser.add_argument(
        'json_path',
        type=str,
        nargs='?',
        default=None,
        help='Path to the output .json file (optional, defaults to input filename with .json extension)'
    )
    
    # Show help if no arguments or --help / -h
    if len(sys.argv) == 1 or '--help' in sys.argv or '-h' in sys.argv:
        parser.print_help()
        sys.exit(0)
    
    args = parser.parse_args()
    
    try:
        pth_to_json(args.pth_path, args.json_path)
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)