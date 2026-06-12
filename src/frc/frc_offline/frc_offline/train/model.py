"""TinyUNet（约 0.4M 参数）+ 健康标量 FiLM 条件化（设计文档 §5.4）。

输出 2 通道：risk logit + confidence logit。
导出侧用 FlatTinyUNet 封装为单一扁平输入/输出（与在线 TrtRiskModel 的
缓冲布局一致：输入 [C*H*W + K]，输出 [risk_logit; sigmoid(conf)] 2*H*W）。
"""

import torch
import torch.nn as nn


def _block(cin, cout, stride=1):
    return nn.Sequential(
        nn.Conv2d(cin, cout, 3, stride=stride, padding=1, bias=False),
        nn.GroupNorm(min(8, cout), cout),
        nn.SiLU(inplace=True),
        nn.Conv2d(cout, cout, 3, padding=1, bias=False),
        nn.GroupNorm(min(8, cout), cout),
        nn.SiLU(inplace=True),
    )


class TinyUNet(nn.Module):
    def __init__(self, in_channels=7, health_dim=6, base=16,
                 use_health_film=True):
        super().__init__()
        self.use_health_film = use_health_film
        self.enc1 = _block(in_channels, base)            # 120
        self.enc2 = _block(base, base * 2, stride=2)     # 60
        self.enc3 = _block(base * 2, base * 4, stride=2) # 30 (bottleneck 64)

        if use_health_film:
            self.film = nn.Sequential(
                nn.Linear(health_dim, 32), nn.SiLU(),
                nn.Linear(32, base * 8),                 # gamma + beta
            )

        self.up2 = nn.ConvTranspose2d(base * 4, base * 2, 2, stride=2)
        self.dec2 = _block(base * 4, base * 2)
        self.up1 = nn.ConvTranspose2d(base * 2, base, 2, stride=2)
        self.dec1 = _block(base * 2, base)
        self.head = nn.Conv2d(base, 2, 1)                # risk logit + conf logit

    def forward(self, bev, health):
        e1 = self.enc1(bev)
        e2 = self.enc2(e1)
        e3 = self.enc3(e2)
        if self.use_health_film:
            gb = self.film(health)
            gamma, beta = gb.chunk(2, dim=1)
            e3 = e3 * (1.0 + gamma[:, :, None, None]) + beta[:, :, None, None]
        d2 = self.dec2(torch.cat([self.up2(e3), e2], dim=1))
        d1 = self.dec1(torch.cat([self.up1(d2), e1], dim=1))
        return self.head(d1)                             # (B, 2, H, W)


class FlatTinyUNet(nn.Module):
    """ONNX/TRT 部署封装：单一扁平输入 -> 扁平输出，温度内嵌。"""

    def __init__(self, model: TinyUNet, in_channels, grid, health_dim,
                 temperature=1.0):
        super().__init__()
        self.model = model
        self.c, self.n, self.k = in_channels, grid, health_dim
        self.register_buffer("temperature",
                             torch.tensor(float(temperature)))

    def forward(self, flat):
        bev = flat[: self.c * self.n * self.n].reshape(
            1, self.c, self.n, self.n)
        health = flat[self.c * self.n * self.n:].reshape(1, self.k)
        out = self.model(bev, health)[0]
        risk_logit = out[0] / self.temperature
        conf = torch.sigmoid(out[1])
        return torch.cat([risk_logit.reshape(-1), conf.reshape(-1)])


def count_params(model) -> int:
    return sum(p.numel() for p in model.parameters())
