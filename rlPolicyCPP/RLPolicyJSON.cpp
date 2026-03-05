#include "RLPolicyJSON.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <cstring>
#include <regex>
#include <set>
#include <vector>
#include <cstdint>

// Base64 decoding helper functions
static inline bool is_base64(unsigned char c) {
    return (isalnum(c) || (c == '+') || (c == '/'));
}

static std::vector<uint8_t> base64_decode(const std::string& encoded_string) {
    const std::string chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::vector<uint8_t> result;
    int in_len = static_cast<int>(encoded_string.size());
    int i = 0;
    int in = 0;
    unsigned char char_array_4[4], char_array_3[3];
    
    while (in_len-- && (encoded_string[in] != '=') && is_base64(encoded_string[in])) {
        char_array_4[i++] = encoded_string[in]; in++;
        if (i == 4) {
            for (i = 0; i < 4; i++) {
                size_t pos = chars.find(char_array_4[i]);
                if (pos != std::string::npos) {
                    char_array_4[i] = static_cast<unsigned char>(pos);
                } else {
                    char_array_4[i] = 0;
                }
            }
            
            char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
            char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
            char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];
            
            for (i = 0; (i < 3); i++)
                result.push_back(char_array_3[i]);
            i = 0;
        }
    }
    
    if (i) {
        for (int j = i; j < 4; j++)
            char_array_4[j] = 0;
        
        for (int j = 0; j < 4; j++) {
            size_t pos = chars.find(char_array_4[j]);
            if (pos != std::string::npos) {
                char_array_4[j] = static_cast<unsigned char>(pos);
            } else {
                char_array_4[j] = 0;
            }
        }
        
        char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
        char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
        char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];
        
        for (int j = 0; (j < i - 1); j++) result.push_back(char_array_3[j]);
    }
    
    return result;
}

// Simple JSON value storage
struct RLPolicyJSON::JSONValue {
    enum Type { OBJECT, ARRAY, STRING, NUMBER, BOOL, NULL_VAL };
    Type type;
    std::map<std::string, std::shared_ptr<JSONValue>> object;
    std::vector<std::shared_ptr<JSONValue>> array;
    std::string string_val;
    double number_val;
    bool bool_val;
    
    JSONValue() : type(NULL_VAL), number_val(0.0), bool_val(false) {}
};

// Simple JSON parser (handles the specific structure from pth2json.py)
class SimpleJSONParser {
private:
    std::string content;
    size_t pos;
    
    void skip_whitespace() {
        while (pos < content.size() && std::isspace(content[pos])) pos++;
    }
    
    std::shared_ptr<RLPolicyJSON::JSONValue> parse_value() {
        skip_whitespace();
        if (pos >= content.size()) return nullptr;
        
        char c = content[pos];
        if (c == '{') return parse_object();
        if (c == '[') return parse_array();
        if (c == '"') return parse_string();
        if (c == 't' || c == 'f') return parse_bool();
        if (c == 'n') return parse_null();
        if (c == '-' || (c >= '0' && c <= '9')) return parse_number();
        return nullptr;
    }
    
    std::shared_ptr<RLPolicyJSON::JSONValue> parse_object() {
        auto obj = std::make_shared<RLPolicyJSON::JSONValue>();
        obj->type = RLPolicyJSON::JSONValue::OBJECT;
        
        pos++; // skip '{'
        skip_whitespace();
        
        while (pos < content.size() && content[pos] != '}') {
            skip_whitespace();
            if (content[pos] == '}') break;
            
            // Parse key
            auto key_val = parse_string();
            if (!key_val) break;
            std::string key = key_val->string_val;
            
            skip_whitespace();
            if (pos >= content.size() || content[pos] != ':') break;
            pos++; // skip ':'
            
            // Parse value
            auto val = parse_value();
            if (val) {
                obj->object[key] = val;
            }
            
            skip_whitespace();
            if (pos < content.size() && content[pos] == ',') {
                pos++; // skip ','
            }
        }
        
        if (pos < content.size() && content[pos] == '}') pos++;
        return obj;
    }
    
    std::shared_ptr<RLPolicyJSON::JSONValue> parse_array() {
        auto arr = std::make_shared<RLPolicyJSON::JSONValue>();
        arr->type = RLPolicyJSON::JSONValue::ARRAY;
        
        pos++; // skip '['
        skip_whitespace();
        
        while (pos < content.size() && content[pos] != ']') {
            skip_whitespace();
            if (content[pos] == ']') break;
            
            auto val = parse_value();
            if (val) {
                arr->array.push_back(val);
            }
            
            skip_whitespace();
            if (pos < content.size() && content[pos] == ',') {
                pos++; // skip ','
            }
        }
        
        if (pos < content.size() && content[pos] == ']') pos++;
        return arr;
    }
    
    std::shared_ptr<RLPolicyJSON::JSONValue> parse_string() {
        auto str = std::make_shared<RLPolicyJSON::JSONValue>();
        str->type = RLPolicyJSON::JSONValue::STRING;
        
        if (pos >= content.size() || content[pos] != '"') return nullptr;
        pos++; // skip opening quote
        
        std::stringstream ss;
        while (pos < content.size() && content[pos] != '"') {
            if (content[pos] == '\\' && pos + 1 < content.size()) {
                pos++;
                if (content[pos] == 'n') ss << '\n';
                else if (content[pos] == 't') ss << '\t';
                else if (content[pos] == 'r') ss << '\r';
                else if (content[pos] == '\\') ss << '\\';
                else if (content[pos] == '"') ss << '"';
                else ss << content[pos];
            } else {
                ss << content[pos];
            }
            pos++;
        }
        
        if (pos < content.size() && content[pos] == '"') pos++;
        str->string_val = ss.str();
        return str;
    }
    
    std::shared_ptr<RLPolicyJSON::JSONValue> parse_number() {
        auto num = std::make_shared<RLPolicyJSON::JSONValue>();
        num->type = RLPolicyJSON::JSONValue::NUMBER;
        
        size_t start = pos;
        
        if (pos < content.size() && content[pos] == '-') pos++;
        while (pos < content.size() && (std::isdigit(content[pos]) || content[pos] == '.' || content[pos] == 'e' || content[pos] == 'E' || content[pos] == '+' || content[pos] == '-')) {
            pos++;
        }
        
        std::string num_str = content.substr(start, pos - start);
        num->number_val = std::stod(num_str);
        return num;
    }
    
    std::shared_ptr<RLPolicyJSON::JSONValue> parse_bool() {
        auto b = std::make_shared<RLPolicyJSON::JSONValue>();
        b->type = RLPolicyJSON::JSONValue::BOOL;
        
        if (content.substr(pos, 4) == "true") {
            b->bool_val = true;
            pos += 4;
        } else if (content.substr(pos, 5) == "false") {
            b->bool_val = false;
            pos += 5;
        }
        return b;
    }
    
    std::shared_ptr<RLPolicyJSON::JSONValue> parse_null() {
        if (content.substr(pos, 4) == "null") {
            pos += 4;
            auto n = std::make_shared<RLPolicyJSON::JSONValue>();
            n->type = RLPolicyJSON::JSONValue::NULL_VAL;
            return n;
        }
        return nullptr;
    }
    
public:
    std::shared_ptr<RLPolicyJSON::JSONValue> parse(const std::string& json_str) {
        content = json_str;
        pos = 0;
        return parse_value();
    }
};

RLPolicyJSON::RLPolicyJSON() : obs_size_(0), action_size_(0), gru_hidden_size_(0), gru_input_size_(0), has_rnn_(true), has_obs_normalizer_(true), use_swarm_encoder_(false), swarm_self_obs_dim_(4), swarm_my_destination_dim_(2) {
    json_data_ = std::make_shared<JSONValue>();
}

RLPolicyJSON::~RLPolicyJSON() = default;

bool RLPolicyJSON::parse_json_file(const std::string& json_path) {
    std::ifstream file(json_path);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open JSON file: " << json_path << std::endl;
        return false;
    }
    
    std::stringstream buffer;
    buffer << file.rdbuf();
    file.close();
    
    SimpleJSONParser parser;
    json_data_ = parser.parse(buffer.str());
    
    if (!json_data_ || json_data_->type != JSONValue::OBJECT) {
        std::cerr << "Error: Failed to parse JSON file" << std::endl;
        return false;
    }
    
    return true;
}

bool RLPolicyJSON::extract_tensor(const std::string& key, std::vector<float>& data, std::vector<int>& shape) {
    // Navigate through JSON structure: weights.key.data and weights.key.shape
    // Handle both cases:
    // 1. Direct: {"weights": {"core.core.weight_ih_l0": {...}}}
    // 2. With model wrapper (old format): {"model": {"weights": {...}}}
    if (json_data_->type != JSONValue::OBJECT) return false;
    
    std::shared_ptr<JSONValue> weights;
    
    // First try direct weights access
    auto weights_it = json_data_->object.find("weights");
    if (weights_it != json_data_->object.end()) {
        weights = weights_it->second;
    } else {
        // Check if there's a "model" wrapper (for old JSON files)
        auto model_it = json_data_->object.find("model");
        if (model_it != json_data_->object.end() && model_it->second->type == JSONValue::OBJECT) {
            auto model_weights_it = model_it->second->object.find("weights");
            if (model_weights_it != model_it->second->object.end()) {
                weights = model_weights_it->second;
            } else {
                // Maybe the model itself contains the weights directly
                weights = model_it->second;
            }
        }
    }
    
    if (!weights || weights->type != JSONValue::OBJECT) {
        std::cerr << "Debug: 'weights' key not found in root object or under 'model'" << std::endl;
        std::cerr << "Debug: Root object keys:" << std::endl;
        for (const auto& pair : json_data_->object) {
            std::cerr << "  - " << pair.first << std::endl;
        }
        return false;
    }
    
    auto key_it = weights->object.find(key);
    if (key_it == weights->object.end()) {
        std::cerr << "Debug: Key '" << key << "' not found in weights object" << std::endl;
        std::cerr << "Debug: Available keys (first 20):" << std::endl;
        int count = 0;
        for (const auto& pair : weights->object) {
            if (count++ < 20) {
                std::cerr << "  - " << pair.first << std::endl;
            }
        }
        if (weights->object.size() > 20) {
            std::cerr << "  ... (total " << weights->object.size() << " keys)" << std::endl;
        }
        // Try to find keys that contain "core" or "weight_ih"
        std::cerr << "Debug: Searching for keys containing 'core' or 'weight_ih':" << std::endl;
        bool found_related = false;
        for (const auto& pair : weights->object) {
            if (pair.first.find("core") != std::string::npos || 
                pair.first.find("weight_ih") != std::string::npos) {
                std::cerr << "  Found: " << pair.first << std::endl;
                found_related = true;
            }
        }
        if (!found_related) {
            std::cerr << "  (no keys containing 'core' or 'weight_ih' found)" << std::endl;
        }
        return false;
    }
    
    auto tensor_obj = key_it->second;
    if (tensor_obj->type != JSONValue::OBJECT) 
        return false;
    
    // Extract shape
    auto shape_it = tensor_obj->object.find("shape");
    if (shape_it == tensor_obj->object.end()) return false;
    auto shape_val = shape_it->second;
    if (shape_val->type != JSONValue::ARRAY) return false;
    
    shape.clear();
    for (auto& dim : shape_val->array) {
        if (dim->type == JSONValue::NUMBER) {
            shape.push_back(static_cast<int>(dim->number_val));
        }
    }
    
    // Extract data - check if it's base64 encoded or array format
    auto data_it = tensor_obj->object.find("data");
    if (data_it == tensor_obj->object.end()) return false;
    auto data_val = data_it->second;
    
    // Check encoding format
    auto encoding_it = tensor_obj->object.find("encoding");
    bool is_base64_encoded = false;
    if (encoding_it != tensor_obj->object.end() && 
        encoding_it->second->type == JSONValue::STRING &&
        encoding_it->second->string_val == "base64") {
        is_base64_encoded = true;
    }
    
    data.clear();
    
    if (is_base64_encoded) {
        // Decode base64 string to bytes, then convert to floats
        if (data_val->type != JSONValue::STRING) {
            std::cerr << "Error: Expected string for base64 data" << std::endl;
            return false;
        }
        
        // Decode base64
        std::vector<uint8_t> decoded_bytes = base64_decode(data_val->string_val);
        
        // Get dtype to determine element size
        auto dtype_it = tensor_obj->object.find("dtype");
        std::string dtype = "float32";  // default
        if (dtype_it != tensor_obj->object.end() && dtype_it->second->type == JSONValue::STRING) {
            dtype = dtype_it->second->string_val;
        }
        
        // Determine element size based on dtype
        size_t element_size = 4;  // default to float32
        if (dtype.find("float64") != std::string::npos || dtype.find("double") != std::string::npos) {
            element_size = 8;
        } else if (dtype.find("float32") != std::string::npos || dtype.find("float") != std::string::npos) {
            element_size = 4;
        } else if (dtype.find("float16") != std::string::npos || dtype.find("half") != std::string::npos) {
            element_size = 2;
        }
        
        // Convert bytes to floats (assuming little-endian, native byte order)
        size_t num_elements = decoded_bytes.size() / element_size;
        data.reserve(num_elements);
        
        if (element_size == 4) {
            // float32 - direct copy is most efficient
            data.resize(num_elements);
            std::memcpy(data.data(), decoded_bytes.data(), num_elements * sizeof(float));
        } else {
            // For other sizes, convert element by element
            for (size_t i = 0; i < num_elements; i++) {
                float val = 0.0f;
                if (element_size == 8) {
                    // float64 -> convert to float32
                    double val_double = 0.0;
                    std::memcpy(&val_double, &decoded_bytes[i * 8], sizeof(double));
                    val = static_cast<float>(val_double);
                } else if (element_size == 2) {
                    // float16 -> convert to float32 (simplified, may need proper half precision conversion)
                    uint16_t bits = 0;
                    std::memcpy(&bits, &decoded_bytes[i * 2], sizeof(uint16_t));
                    // Simple conversion (for proper half precision, use a library)
                    val = static_cast<float>(bits) / 65536.0f;  // rough approximation
                }
                data.push_back(val);
            }
        }
    } else {
        // Old format: extract from nested array
        extract_flattened_array(data_val, data);
    }
    
    return true;
}

void RLPolicyJSON::extract_flattened_array(std::shared_ptr<JSONValue> val, std::vector<float>& result) {
    if (!val) return;
    if (val->type == JSONValue::ARRAY) {
        for (auto& item : val->array) {
            extract_flattened_array(item, result);
        }
    } else if (val->type == JSONValue::NUMBER) {
        result.push_back(static_cast<float>(val->number_val));
    }
}

static std::shared_ptr<RLPolicyJSON::JSONValue> get_weights_object(RLPolicyJSON::JSONValue* json_data) {
    if (!json_data || json_data->type != RLPolicyJSON::JSONValue::OBJECT) return nullptr;
    auto it = json_data->object.find("weights");
    if (it != json_data->object.end() && it->second && it->second->type == RLPolicyJSON::JSONValue::OBJECT)
        return it->second;
    auto model_it = json_data->object.find("model");
    if (model_it != json_data->object.end() && model_it->second && model_it->second->type == RLPolicyJSON::JSONValue::OBJECT) {
        auto w_it = model_it->second->object.find("weights");
        if (w_it != model_it->second->object.end() && w_it->second && w_it->second->type == RLPolicyJSON::JSONValue::OBJECT)
            return w_it->second;
        return model_it->second;
    }
    return nullptr;
}

bool RLPolicyJSON::has_key_in_weights(const std::string& key) {
    auto weights = get_weights_object(json_data_.get());
    return weights && weights->object.find(key) != weights->object.end();
}

bool RLPolicyJSON::is_swarm_encoder_json() const {
    return const_cast<RLPolicyJSON*>(this)->has_key_in_weights("actor_encoder.self_goal_encoder.0.weight");
}

bool RLPolicyJSON::find_mlp_linear_indices(std::vector<int>& indices) {
    indices.clear();
    if (json_data_->type != JSONValue::OBJECT) return false;
    
    auto weights = get_weights_object(json_data_.get());
    if (!weights) return false;
    
    std::set<int> index_set;
    std::string prefix = "encoder.encoders.obs.mlp_head.";
    
    for (const auto& pair : weights->object) {
        const std::string& key = pair.first;
        if (key.size() >= prefix.size() && key.compare(0, prefix.size(), prefix) == 0 && key.find(".weight") != std::string::npos) {
            // Extract index: encoder.encoders.obs.mlp_head.{idx}.weight
            size_t idx_start = prefix.length();
            size_t idx_end = key.find('.', idx_start);
            if (idx_end != std::string::npos) {
                try {
                    int idx = std::stoi(key.substr(idx_start, idx_end - idx_start));
                    index_set.insert(idx);
                } catch (...) {
                    continue;
                }
            }
        }
    }
    
    indices.assign(index_set.begin(), index_set.end());
    std::sort(indices.begin(), indices.end());
    return true;
}

bool RLPolicyJSON::load_normalizer_from_json() {
    std::string mean_key = "obs_normalizer.running_mean_std.running_mean_std.obs.running_mean";
    std::string var_key = "obs_normalizer.running_mean_std.running_mean_std.obs.running_var";
    
    std::vector<float> mean_data, var_data;
    std::vector<int> mean_shape, var_shape;
    
    if (extract_tensor(mean_key, mean_data, mean_shape) && 
        extract_tensor(var_key, var_data, var_shape)) {
        obs_mean_ = mean_data;
        obs_var_ = var_data;
        obs_size_ = mean_shape.empty() ? static_cast<int>(mean_data.size()) : mean_shape[0];
        has_obs_normalizer_ = true;
        // For swarm encoder: truncate if obs_mean length doesn't match encoder structure (self_goal + n*4)
        if (is_swarm_encoder_json() && obs_size_ > 0) {
            std::vector<float> self_goal_w;
            std::vector<int> self_goal_shape;
            if (extract_tensor("actor_encoder.self_goal_encoder.0.weight", self_goal_w, self_goal_shape) &&
                self_goal_shape.size() >= 2) {
                int self_goal_in = self_goal_shape[1];
                const int single_neighbor = 4;
                int remainder = (obs_size_ - self_goal_in) % single_neighbor;
                if (remainder != 0) {
                    int valid_len = self_goal_in + ((obs_size_ - self_goal_in) / single_neighbor) * single_neighbor;
                    obs_mean_.resize(valid_len);
                    obs_var_.resize(valid_len);
                    obs_size_ = valid_len;
                }
            }
        }
        return true;
    }
    
    has_obs_normalizer_ = false;
    // Fallback: infer from encoder (MLP or swarm)
    std::vector<int> indices;
    if (find_mlp_linear_indices(indices) && !indices.empty()) {
        std::string first_w_key = "encoder.encoders.obs.mlp_head." + std::to_string(indices[0]) + ".weight";
        std::vector<float> first_w_data;
        std::vector<int> first_w_shape;
        if (extract_tensor(first_w_key, first_w_data, first_w_shape) && first_w_shape.size() >= 2) {
            obs_size_ = first_w_shape[1];
            obs_mean_.assign(obs_size_, 0.0f);
            obs_var_.assign(obs_size_, 1.0f);
            return true;
        }
    }
    if (is_swarm_encoder_json()) {
        // Infer obs_size from swarm encoder weights (supports 2+2 and 4+2 self_goal layouts)
        std::vector<float> self_goal_w;
        std::vector<int> self_goal_shape;
        if (extract_tensor("actor_encoder.self_goal_encoder.0.weight", self_goal_w, self_goal_shape) &&
            self_goal_shape.size() >= 2) {
            int self_goal_in = self_goal_shape[1];
            const int single_neighbor = 4;
            // obs = self_goal_in + num_neighbors*4; default 4 agents -> 3 neighbors
            obs_size_ = self_goal_in + single_neighbor * 3;
        } else {
            obs_size_ = 18;  // fallback
        }
        obs_mean_.assign(obs_size_, 0.0f);
        obs_var_.assign(obs_size_, 1.0f);
        return true;
    }
    
    // Last resort
    obs_size_ = 1;
    obs_mean_.assign(1, 0.0f);
    obs_var_.assign(1, 1.0f);
    return true;
}

bool RLPolicyJSON::build_encoder_from_json(Activation nonlinearity) {
    encoder_layers_.clear();
    
    std::vector<int> indices;
    if (!find_mlp_linear_indices(indices)) {
        // No encoder layers - use identity
        return true;
    }
    
    for (int idx : indices) {
        std::string w_key = "encoder.encoders.obs.mlp_head." + std::to_string(idx) + ".weight";
        std::string b_key = "encoder.encoders.obs.mlp_head." + std::to_string(idx) + ".bias";
        
        std::vector<float> w_data, b_data;
        std::vector<int> w_shape, b_shape;
        
        if (!extract_tensor(w_key, w_data, w_shape) || 
            !extract_tensor(b_key, b_data, b_shape)) {
            std::cerr << "Warning: Could not load encoder layer " << idx << std::endl;
            continue;
        }
        
        if (w_shape.size() != 2) {
            std::cerr << "Warning: Invalid weight shape for layer " << idx << std::endl;
            continue;
        }
        
        EncoderLayer layer;
        layer.linear.in_features = w_shape[1];
        layer.linear.out_features = w_shape[0];
        layer.activation = nonlinearity;
        
        // Reshape weight matrix: [out_features, in_features]
        layer.linear.weights.resize(w_shape[0]);
        for (int i = 0; i < w_shape[0]; i++) {
            layer.linear.weights[i].resize(w_shape[1]);
            for (int j = 0; j < w_shape[1]; j++) {
                layer.linear.weights[i][j] = w_data[i * w_shape[1] + j];
            }
        }
        
        layer.linear.bias = b_data;
        encoder_layers_.push_back(layer);
    }
    
    return true;
}

bool RLPolicyJSON::build_swarm_encoder_from_json(Activation nonlinearity) {
    use_swarm_encoder_ = true;
    swarm_self_goal_layers_.clear();
    swarm_embedding_layers_.clear();
    swarm_value_layers_.clear();
    swarm_attention_layers_.clear();
    auto load_one = [this, nonlinearity](const std::string& prefix, const std::vector<int>& indices, bool last_has_act) -> std::vector<EncoderLayer> {
        std::vector<EncoderLayer> layers;
        for (size_t i = 0; i < indices.size(); i++) {
            int idx = indices[i];
            std::string w_key = prefix + std::to_string(idx) + ".weight";
            std::string b_key = prefix + std::to_string(idx) + ".bias";
            std::vector<float> w_data, b_data;
            std::vector<int> w_shape, b_shape;
            if (!extract_tensor(w_key, w_data, w_shape) || !extract_tensor(b_key, b_data, b_shape) || w_shape.size() != 2)
                return {};
            EncoderLayer layer;
            layer.linear.in_features = w_shape[1];
            layer.linear.out_features = w_shape[0];
            layer.activation = (i < indices.size() - 1 || last_has_act) ? nonlinearity : Activation::RELU;
            layer.linear.weights.resize(w_shape[0]);
            for (int r = 0; r < w_shape[0]; r++) {
                layer.linear.weights[r].resize(w_shape[1]);
                for (int c = 0; c < w_shape[1]; c++)
                    layer.linear.weights[r][c] = w_data[r * w_shape[1] + c];
            }
            layer.linear.bias = b_data;
            layers.push_back(layer);
        }
        return layers;
    };
    std::string pre = "actor_encoder.";
    swarm_self_goal_layers_ = load_one(pre + "self_goal_encoder.", {0, 2}, true);
    swarm_embedding_layers_ = load_one(pre + "neighbor_encoder.embedding_mlp.", {0, 2}, true);
    swarm_value_layers_ = load_one(pre + "neighbor_encoder.value_mlp.", {0, 2}, true);
    swarm_attention_layers_ = load_one(pre + "neighbor_encoder.attention_mlp.", {0, 2, 4}, false);
    if (swarm_self_goal_layers_.empty() || swarm_embedding_layers_.empty() || swarm_value_layers_.empty() || swarm_attention_layers_.size() != 3u)
        return false;
    // Infer self_obs and my_destination dims from encoder (supports 2+2 and 4+2 layouts)
    const int single_neighbor = 4;
    int self_goal_in = swarm_self_goal_layers_[0].linear.in_features;
    int embed_in = swarm_embedding_layers_[0].linear.in_features;
    swarm_self_obs_dim_ = embed_in - single_neighbor;
    swarm_my_destination_dim_ = self_goal_in - swarm_self_obs_dim_;
    return true;
}

bool RLPolicyJSON::load_gru_from_json() {
    std::string w_ih_key = "core.core.weight_ih_l0";
    std::string w_hh_key = "core.core.weight_hh_l0";
    std::string b_ih_key = "core.core.bias_ih_l0";
    std::string b_hh_key = "core.core.bias_hh_l0";
    
    std::vector<float> w_ih_data, w_hh_data, b_ih_data, b_hh_data;
    std::vector<int> w_ih_shape, w_hh_shape, b_ih_shape, b_hh_shape;
    
    if (!extract_tensor(w_ih_key, w_ih_data, w_ih_shape) ||
        !extract_tensor(w_hh_key, w_hh_data, w_hh_shape) ||
        !extract_tensor(b_ih_key, b_ih_data, b_ih_shape) ||
        !extract_tensor(b_hh_key, b_hh_data, b_hh_shape)) {
        return false;
    }
    
    // GRU weights: [3*hidden_size, input_size] for weight_ih
    //              [3*hidden_size, hidden_size] for weight_hh
    gru_weights_.hidden_size = w_hh_shape[1];
    gru_weights_.input_size = w_ih_shape[1];
    
    // Reshape weight_ih: [3*hidden, input]
    int rows_ih = w_ih_shape[0];
    int cols_ih = w_ih_shape[1];
    gru_weights_.weight_ih.resize(rows_ih);
    for (int i = 0; i < rows_ih; i++) {
        gru_weights_.weight_ih[i].resize(cols_ih);
        for (int j = 0; j < cols_ih; j++) {
            gru_weights_.weight_ih[i][j] = w_ih_data[i * cols_ih + j];
        }
    }
    
    // Reshape weight_hh: [3*hidden, hidden]
    int rows_hh = w_hh_shape[0];
    int cols_hh = w_hh_shape[1];
    gru_weights_.weight_hh.resize(rows_hh);
    for (int i = 0; i < rows_hh; i++) {
        gru_weights_.weight_hh[i].resize(cols_hh);
        for (int j = 0; j < cols_hh; j++) {
            gru_weights_.weight_hh[i][j] = w_hh_data[i * cols_hh + j];
        }
    }
    
    gru_weights_.bias_ih = b_ih_data;
    gru_weights_.bias_hh = b_hh_data;
    gru_hidden_size_ = gru_weights_.hidden_size;
    
    // Determine input size from encoder
    if (!encoder_layers_.empty()) {
        gru_input_size_ = encoder_layers_.back().linear.out_features;
    } else {
        gru_input_size_ = gru_weights_.input_size;
    }
    
    return true;
}

bool RLPolicyJSON::load_dist_linear_from_json() {
    std::string w_key = "action_parameterization.distribution_linear.weight";
    std::string b_key = "action_parameterization.distribution_linear.bias";
    
    std::vector<float> w_data, b_data;
    std::vector<int> w_shape, b_shape;
    
    if (!extract_tensor(w_key, w_data, w_shape) ||
        !extract_tensor(b_key, b_data, b_shape)) {
        return false;
    }
    
    dist_linear_.in_features = w_shape[1];
    dist_linear_.out_features = w_shape[0];
    action_size_ = w_shape[0];
    
    // Reshape weight matrix: [out_features, in_features]
    dist_linear_.weights.resize(w_shape[0]);
    for (int i = 0; i < w_shape[0]; i++) {
        dist_linear_.weights[i].resize(w_shape[1]);
        for (int j = 0; j < w_shape[1]; j++) {
            dist_linear_.weights[i][j] = w_data[i * w_shape[1] + j];
        }
    }
    
    dist_linear_.bias = b_data;

    // Load learned_stddev if present (adaptive_stddev=False)
    std::string learned_key = "action_parameterization.learned_stddev";
    std::vector<float> learned_data;
    std::vector<int> learned_shape;
    if (extract_tensor(learned_key, learned_data, learned_shape)) {
        learned_stddev_ = learned_data;
    } else {
        learned_stddev_.clear();
    }
    return true;
}

bool RLPolicyJSON::load_from_json(const std::string& json_path, Activation nonlinearity) {
    if (!parse_json_file(json_path)) {
        return false;
    }
    
    if (!load_normalizer_from_json()) {
        std::cerr << "Warning: Could not load normalizer, using defaults" << std::endl;
    }
    
    if (is_swarm_encoder_json()) {
        if (!build_swarm_encoder_from_json(nonlinearity)) {
            std::cerr << "Error: Could not build swarm encoder" << std::endl;
            return false;
        }
    } else if (!build_encoder_from_json(nonlinearity)) {
        std::cerr << "Error: Could not build encoder" << std::endl;
        return false;
    }
    
    if (!load_dist_linear_from_json()) {
        std::cerr << "Error: Could not load distribution linear" << std::endl;
        return false;
    }
    
    if (!load_gru_from_json()) {
        has_rnn_ = false;
        gru_hidden_size_ = dist_linear_.in_features;
        gru_input_size_ = dist_linear_.in_features;
        std::cerr << "Note: No GRU in checkpoint (use_rnn=False), using encoder output directly" << std::endl;
    }
    
    reset_hidden_state(1);
    return true;
}

float RLPolicyJSON::activation_func(float x, Activation act) const {
    switch (act) {
        case Activation::RELU:
            return std::max(0.0f, x);
        case Activation::ELU:
            return x > 0.0f ? x : (std::exp(x) - 1.0f);
        case Activation::TANH:
            return std::tanh(x);
    }
    return x;
}

std::vector<float> RLPolicyJSON::linear_forward(const LinearLayer& layer, const std::vector<float>& input) const {
    std::vector<float> output(layer.out_features, 0.0f);
    
    for (int i = 0; i < layer.out_features; i++) {
        output[i] = layer.bias[i];
        for (int j = 0; j < layer.in_features; j++) {
            output[i] += layer.weights[i][j] * input[j];
        }
    }
    
    return output;
}

std::vector<float> RLPolicyJSON::encoder_forward(const std::vector<float>& input) const {
    if (use_swarm_encoder_)
        return swarm_encoder_forward(input);
    std::vector<float> x = input;
    for (const auto& layer : encoder_layers_) {
        x = linear_forward(layer.linear, x);
        for (float& val : x) {
            val = activation_func(val, layer.activation);
        }
    }
    return x;
}

std::vector<float> RLPolicyJSON::swarm_encoder_forward(const std::vector<float>& input) const {
    const int self_obs_dim = swarm_self_obs_dim_;
    const int my_destination_dim = swarm_my_destination_dim_;
    const int single_neighbor_dim = 4;
    if (input.size() < static_cast<size_t>(self_obs_dim + my_destination_dim))
        return std::vector<float>(dist_linear_.in_features, 0.0f);
    size_t idx = 0;
    std::vector<float> self_obs(input.begin() + idx, input.begin() + idx + self_obs_dim);
    idx += self_obs_dim;
    std::vector<float> my_destination(input.begin() + idx, input.begin() + idx + my_destination_dim);
    idx += my_destination_dim;
    std::vector<float> neighbor_obs_flat(input.begin() + idx, input.end());
    int num_neighbors = static_cast<int>(neighbor_obs_flat.size()) / single_neighbor_dim;
    if (num_neighbors <= 0) {
        std::vector<float> self_goal_in(self_obs);
        self_goal_in.insert(self_goal_in.end(), my_destination.begin(), my_destination.end());
        std::vector<float> self_goal_embed = self_goal_in;
        for (const auto& layer : swarm_self_goal_layers_) {
            self_goal_embed = linear_forward(layer.linear, self_goal_embed);
            for (float& v : self_goal_embed) v = activation_func(v, layer.activation);
        }
        std::vector<float> neighbor_embed(swarm_value_layers_.back().linear.out_features, 0.0f);
        self_goal_embed.insert(self_goal_embed.end(), neighbor_embed.begin(), neighbor_embed.end());
        return self_goal_embed;
    }
    std::vector<float> self_goal_in(self_obs);
    self_goal_in.insert(self_goal_in.end(), my_destination.begin(), my_destination.end());
    std::vector<float> self_goal_embed = self_goal_in;
    for (const auto& layer : swarm_self_goal_layers_) {
        self_goal_embed = linear_forward(layer.linear, self_goal_embed);
        for (float& v : self_goal_embed) v = activation_func(v, layer.activation);
    }
    int hidden_size = swarm_embedding_layers_.back().linear.out_features;
    std::vector<float> embeddings;
    for (int n = 0; n < num_neighbors; n++) {
        std::vector<float> concat(self_obs);
        for (int j = 0; j < single_neighbor_dim; j++)
            concat.push_back(neighbor_obs_flat[n * single_neighbor_dim + j]);
        std::vector<float> e = concat;
        for (const auto& layer : swarm_embedding_layers_) {
            e = linear_forward(layer.linear, e);
            for (float& v : e) v = activation_func(v, layer.activation);
        }
        embeddings.insert(embeddings.end(), e.begin(), e.end());
    }
    std::vector<float> values;
    for (int n = 0; n < num_neighbors; n++) {
        std::vector<float> e(embeddings.begin() + n * hidden_size, embeddings.begin() + (n + 1) * hidden_size);
        std::vector<float> v = e;
        for (const auto& layer : swarm_value_layers_) {
            v = linear_forward(layer.linear, v);
            for (float& x : v) x = activation_func(x, layer.activation);
        }
        values.insert(values.end(), v.begin(), v.end());
    }
    std::vector<float> mean_embed(hidden_size, 0.0f);
    for (int n = 0; n < num_neighbors; n++)
        for (int h = 0; h < hidden_size; h++)
            mean_embed[h] += embeddings[n * hidden_size + h];
    for (float& v : mean_embed) v /= num_neighbors;
    std::vector<float> attn_scores;
    for (int n = 0; n < num_neighbors; n++) {
        std::vector<float> cat(embeddings.begin() + n * hidden_size, embeddings.begin() + (n + 1) * hidden_size);
        cat.insert(cat.end(), mean_embed.begin(), mean_embed.end());
        std::vector<float> a = cat;
        for (size_t L = 0; L < swarm_attention_layers_.size(); L++) {
            a = linear_forward(swarm_attention_layers_[L].linear, a);
            if (L < swarm_attention_layers_.size() - 1)
                for (float& v : a) v = activation_func(v, swarm_attention_layers_[L].activation);
        }
        attn_scores.push_back(a[0]);
    }
    // Numerically stable softmax (match PyTorch F.softmax): subtract max before exp
    float max_score = attn_scores[0];
    for (size_t i = 1; i < attn_scores.size(); i++)
        if (attn_scores[i] > max_score) max_score = attn_scores[i];
    float sum_exp = 0.0f;
    for (float& s : attn_scores) {
        s = std::exp(s - max_score);
        sum_exp += s;
    }
    for (float& s : attn_scores) s /= sum_exp;
    std::vector<float> neighbor_embed(hidden_size, 0.0f);
    for (int n = 0; n < num_neighbors; n++)
        for (int h = 0; h < hidden_size; h++)
            neighbor_embed[h] += attn_scores[n] * values[n * hidden_size + h];
    self_goal_embed.insert(self_goal_embed.end(), neighbor_embed.begin(), neighbor_embed.end());
    return self_goal_embed;
}

std::vector<float> RLPolicyJSON::normalize_obs(const std::vector<float>& obs) const {
    std::vector<float> normalized(obs.size());
    for (size_t i = 0; i < obs.size(); i++) {
        float std_val = std::sqrt(obs_var_[i] + EPS);
        normalized[i] = (obs[i] - obs_mean_[i]) / std_val;
        normalized[i] = std::max(-5.0f, std::min(5.0f, normalized[i]));
    }
    return normalized;
}

    std::vector<float> RLPolicyJSON::gru_forward(const std::vector<float>& input) {
        // GRU computation: simplified single-step forward
        // For full GRU: h_t = (1 - z_t) * h_{t-1} + z_t * n_t
        // where z_t = sigmoid(W_z @ x + U_z @ h + b_z)  [update gate]
        //       r_t = sigmoid(W_r @ x + U_r @ h + b_r)  [reset gate]
        //       n_t = tanh(W_n @ x + r_t * (U_n @ h) + b_n)  [new gate]
        //
        // IMPORTANT: The tanh activation for the new gate (n) is hardcoded as part of
        // the standard GRU architecture. This is NOT configurable and is independent
        // of the activation function parameter (relu/elu/tanh) used in encoder layers.
        
        (void)input;  // Suppress unused parameter warning if input size matches gru_input_size_
        int hidden = gru_hidden_size_;
    
    // Split weights into z, r, n gates (each is hidden_size rows)
    auto get_gate_weights = [](const std::vector<std::vector<float>>& weights, int gate, int hidden_size) {
        std::vector<std::vector<float>> gate_weights(hidden_size);
        for (int i = 0; i < hidden_size; i++) {
            gate_weights[i] = weights[gate * hidden_size + i];
        }
        return gate_weights;
    };
    
    auto get_gate_bias = [](const std::vector<float>& bias, int gate, int hidden_size) {
        std::vector<float> gate_bias(hidden_size);
        for (int i = 0; i < hidden_size; i++) {
            gate_bias[i] = bias[gate * hidden_size + i];
        }
        return gate_bias;
    };
    
    // Matrix-vector multiply
    auto matvec = [](const std::vector<std::vector<float>>& W, const std::vector<float>& x) {
        std::vector<float> result(W.size(), 0.0f);
        for (size_t i = 0; i < W.size(); i++) {
            for (size_t j = 0; j < x.size(); j++) {
                result[i] += W[i][j] * x[j];
            }
        }
        return result;
    };
    
    // Element-wise operations
    // Note: GRU cell always uses sigmoid for z/r gates and tanh for n gate.
    // This is part of the standard GRU architecture and is NOT configurable.
    // The activation function parameter (relu/elu/tanh) only applies to encoder MLP layers.
    auto sigmoid = [](float x) { return 1.0f / (1.0f + std::exp(-x)); };
    auto tanh_func = [](float x) { return std::tanh(x); };
    
    // Get gate weights and biases
    // IMPORTANT: PyTorch GRU gate order is [reset, update, new], NOT [update, reset, new]!
    // Gate order: [r_gate, z_gate, n_gate]
    auto W_r = get_gate_weights(gru_weights_.weight_ih, 0, hidden);  // RESET gate
    auto W_z = get_gate_weights(gru_weights_.weight_ih, 1, hidden);  // UPDATE gate
    auto W_n = get_gate_weights(gru_weights_.weight_ih, 2, hidden);  // NEW gate
    
    auto U_r = get_gate_weights(gru_weights_.weight_hh, 0, hidden);  // RESET gate
    auto U_z = get_gate_weights(gru_weights_.weight_hh, 1, hidden);  // UPDATE gate
    auto U_n = get_gate_weights(gru_weights_.weight_hh, 2, hidden);  // NEW gate
    
    auto b_r_ih = get_gate_bias(gru_weights_.bias_ih, 0, hidden);  // RESET gate
    auto b_z_ih = get_gate_bias(gru_weights_.bias_ih, 1, hidden);  // UPDATE gate
    auto b_n_ih = get_gate_bias(gru_weights_.bias_ih, 2, hidden);  // NEW gate
    
    auto b_r_hh = get_gate_bias(gru_weights_.bias_hh, 0, hidden);  // RESET gate
    auto b_z_hh = get_gate_bias(gru_weights_.bias_hh, 1, hidden);  // UPDATE gate
    auto b_n_hh = get_gate_bias(gru_weights_.bias_hh, 2, hidden);  // NEW gate
    
    // Current hidden state
    std::vector<float> h_prev = hxs_;
    
    // Compute gates
    std::vector<float> z_t = matvec(W_z, input);
    std::vector<float> r_t = matvec(W_r, input);
    std::vector<float> n_t = matvec(W_n, input);
    
    std::vector<float> z_h = matvec(U_z, h_prev);
    std::vector<float> r_h = matvec(U_r, h_prev);
    std::vector<float> n_h = matvec(U_n, h_prev);
    
    // Add biases and apply activations
    // Note: For z and r gates, biases are simply added: sigmoid(W @ x + U @ h + b_ih + b_hh)
    // For n gate, the formula is: tanh(W_n @ x + b_n_ih + r * (U_n @ h + b_n_hh))
    // This matches PyTorch's GRU implementation
    std::vector<float> z(hidden), r(hidden), n(hidden);
    for (int i = 0; i < hidden; i++) {
        z[i] = sigmoid(z_t[i] + z_h[i] + b_z_ih[i] + b_z_hh[i]);
        r[i] = sigmoid(r_t[i] + r_h[i] + b_r_ih[i] + b_r_hh[i]);
        // New gate: tanh(W_n @ x + b_n_ih + r * (U_n @ h + b_n_hh))
        n[i] = tanh_func(n_t[i] + b_n_ih[i] + r[i] * (n_h[i] + b_n_hh[i]));
    }
    
    // Update hidden state
    // PyTorch GRU uses: h = n + z * (h_prev - n)
    // This is equivalent to: h = (1 - z) * n + z * h_prev
    // NOT the standard: h = (1 - z) * h_prev + z * n
    // These are mathematically equivalent, but PyTorch's implementation uses the first form
    std::vector<float> h_next(hidden);
    for (int i = 0; i < hidden; i++) {
        h_next[i] = n[i] + z[i] * (h_prev[i] - n[i]);
    }
    
    hxs_ = h_next;
    return h_next;
}

std::vector<float> RLPolicyJSON::forward(const std::vector<float>& obs, bool normalized) {
    // Normalize observation (skip when no obs_normalizer in checkpoint)
    std::vector<float> x;
    if (normalized || !has_obs_normalizer_) {
        x = obs;
    } else {
        x = normalize_obs(obs);
    }
    
    // Encoder forward
    x = encoder_forward(x);
    
    // GRU forward (skip when has_rnn_ is false)
    if (has_rnn_)
        x = gru_forward(x);
    
    // Distribution linear
    std::vector<float> params = linear_forward(dist_linear_, x);
    
    std::vector<float> mean, logstd;
    if (!learned_stddev_.empty()) {
        // adaptive_stddev=False: params = mean only, logstd from learned_stddev_
        mean = params;
        logstd = learned_stddev_;
    } else {
        // adaptive_stddev=True: params = [mean, logstd] concatenated
        int action_dim = static_cast<int>(params.size()) / 2;
        mean.assign(params.begin(), params.begin() + action_dim);
        logstd.assign(params.begin() + action_dim, params.end());
    }
    
    std::vector<float> action_logits;
    action_logits.insert(action_logits.end(), mean.begin(), mean.end());
    action_logits.insert(action_logits.end(), logstd.begin(), logstd.end());
    
    return action_logits;
}

void RLPolicyJSON::reset_hidden_state(int batch_size) {
    (void)batch_size;  // Suppress unused parameter warning
    hxs_.assign(gru_hidden_size_, 0.0f);
}

void RLPolicyJSON::set_hidden_state(const std::vector<float>& hxs) {
    if (hxs.size() == static_cast<size_t>(gru_hidden_size_)) {
        hxs_ = hxs;
    } else {
        std::cerr << "Warning: Hidden state size mismatch" << std::endl;
    }
}

std::vector<float> RLPolicyJSON::get_hidden_state() const {
    return hxs_;
}
