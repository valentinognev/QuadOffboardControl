import math
import torch
import torch.nn as nn

EPS = 1e-8

class RLPolicy(nn.Module):
    def __init__(self):
        super().__init__()
        # 1) obs normalizer parameters (identity for 3-dim obs)
        self.obs_mean = nn.Parameter(torch.zeros(3), requires_grad=False)
        self.obs_var  = nn.Parameter(torch.ones(3), requires_grad=False)
        # 2) encoder MLP: 3 → 64 → 64 → 64
        self.encoder = nn.Sequential(
            nn.Linear(3, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, 64)
        )
        # 3) recurrent core: GRU(64 → 512)
        self.core = nn.GRU(input_size=64, hidden_size=512, batch_first=True)
        # 4) Gaussian head: outputs [mean, logstd] for 3-D action
        self.dist_linear = nn.Linear(512, 6)

    def forward(self, obs, hxs):
        """
        obs: Tensor[batch,3]
        hxs: Tensor[1,batch,512] or None
        returns: action_mean[T,2], new_hxs[1,batch,512]
        """
        # normalize (identity over 3 dims)
        x = (obs - self.obs_mean) / torch.sqrt(self.obs_var + EPS)
        # encode
        x = self.encoder(x)                        # [batch,64]
        # recurrent
        x, h_next = self.core(x.unsqueeze(1), hxs) # x: [batch,1,512]
        x = x.squeeze(1)                           # [batch,512]
        # distribution parameters
        params = self.dist_linear(x)               # [batch,6]
        mean, logstd = params.chunk(2, dim=-1)     # each [batch,3]
        return mean, h_next

    @classmethod
    def load_from_checkpoint(cls, path, device="cpu"):
        """
        Load weights for encoder, core, and dist_linear; skip obs normalizer.
        """
        ckpt = torch.load(path, map_location=device)["model"]
        net = cls().to(device)
        # load MLP weights
        net.encoder[0].weight.data.copy_(ckpt["encoder.encoders.obs.mlp_head.0.weight"])
        net.encoder[0].bias.data  .copy_(ckpt["encoder.encoders.obs.mlp_head.0.bias"])
        net.encoder[2].weight.data.copy_(ckpt["encoder.encoders.obs.mlp_head.2.weight"])
        net.encoder[2].bias.data  .copy_(ckpt["encoder.encoders.obs.mlp_head.2.bias"])
        net.encoder[4].weight.data.copy_(ckpt["encoder.encoders.obs.mlp_head.4.weight"])
        net.encoder[4].bias.data  .copy_(ckpt["encoder.encoders.obs.mlp_head.4.bias"])
        # load GRU weights
        net.core.weight_ih_l0.data.copy_(ckpt["core.core.weight_ih_l0"])
        net.core.weight_hh_l0.data.copy_(ckpt["core.core.weight_hh_l0"])
        net.core.bias_ih_l0  .data.copy_(ckpt["core.core.bias_ih_l0"])
        net.core.bias_hh_l0  .data.copy_(ckpt["core.core.bias_hh_l0"])
        # load distribution head
        net.dist_linear.weight.data.copy_(ckpt["action_parameterization.distribution_linear.weight"])
        net.dist_linear.bias.data  .copy_(ckpt["action_parameterization.distribution_linear.bias"])
        return net
