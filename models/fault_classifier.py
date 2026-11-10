"""
Multi-Task Fault Classification & Severity Network (SIH26054 Section 12, 14)
Classifies engine anomalies into specific root causes and estimates degradation severity.
Includes temperature scaling for calibrated confidence estimation.
"""
try:
    import torch
    import torch.nn as nn
    HAS_TORCH = True
    _ModuleBase = nn.Module
except ImportError:
    HAS_TORCH = False
    torch = None
    nn = None
    _ModuleBase = object

import numpy as np
from typing import Dict, List, Tuple, Optional, Any

from .anomaly_autoencoder import RESIDUAL_CHANNELS

FAULT_CLASSES = [
    "HEALTHY",
    "LEAN_MIXTURE_CYL3",
    "RICH_MIXTURE_CYL1",
    "COOLING_DEGRADATION_CYL2",
    "OIL_PRESSURE_LOSS",
    "TURBO_BOOST_DEFICIENCY",
    "CYLINDER_MISFIRE_TIMING",
    "BEARING_WEAR_VIBRATION",
    "SENSOR_FAULT_EGT3",
    "ELECTRICAL_VOLTAGE_SAG"
]

CLASS_TO_IDX = {c: i for i, c in enumerate(FAULT_CLASSES)}
IDX_TO_CLASS = {i: c for i, c in enumerate(FAULT_CLASSES)}

class FaultClassifierNet(_ModuleBase):
    """
    Multi-task Neural Network:
    - Head 1: 10-Class Fault Logits
    - Head 2: Continuous Severity (0.0 to 1.0)
    """
    def __init__(self, input_dim: int = len(RESIDUAL_CHANNELS), num_classes: int = len(FAULT_CLASSES)):
        if HAS_TORCH:
            super().__init__()
            self.backbone = nn.Sequential(
                nn.Linear(input_dim, 64),
                nn.BatchNorm1d(64),
                nn.LeakyReLU(0.1),
                nn.Dropout(0.15),
                nn.Linear(64, 32),
                nn.BatchNorm1d(32),
                nn.LeakyReLU(0.1)
            )
            self.classifier_head = nn.Linear(32, num_classes)
            self.severity_head = nn.Sequential(
                nn.Linear(32, 16),
                nn.LeakyReLU(0.1),
                nn.Linear(16, 1),
                nn.Sigmoid()
            )
            self.temperature = nn.Parameter(torch.ones(1) * 1.2)

    def forward(self, x):
        if not HAS_TORCH:
            return None, None
        features = self.backbone(x)
        logits = self.classifier_head(features)
        scaled_logits = logits / torch.clamp(self.temperature, min=0.1)
        severity = self.severity_head(features)
        return scaled_logits, severity

class FaultDiagnosisEngine:
    """
    Inference Engine for Fault Classification & Severity.
    """
    def __init__(self, model_path: Optional[str] = None):
        if HAS_TORCH:
            self.device = torch.device("cpu")
            self.model = FaultClassifierNet().to(self.device)
            self.is_trained = False
            if model_path:
                self.load(model_path)
        else:
            self.device = None
            self.model = None
            self.is_trained = False

    def diagnose(self, residual_dict: Dict[str, float]) -> Tuple[str, float, float, Dict[str, float]]:
        if not HAS_TORCH or self.model is None or not self.is_trained:
            # Rule-based fallback diagnosis
            res_oil = abs(residual_dict.get("oil_pressure", 0.0))
            res_egt3 = abs(residual_dict.get("egt_cyl_3", 0.0))
            res_cht2 = abs(residual_dict.get("cht_cyl_2", 0.0))
            res_map = abs(residual_dict.get("manifold_pressure", 0.0))
            res_vib = abs(residual_dict.get("vibration_rms", 0.0))

            if res_oil > 3.0:
                top_class = "OIL_PRESSURE_LOSS"
                conf = 94.5
                sev = min(1.0, res_oil / 6.0)
            elif res_egt3 > 3.0:
                top_class = "LEAN_MIXTURE_CYL3"
                conf = 92.0
                sev = min(1.0, res_egt3 / 6.0)
            elif res_cht2 > 3.0:
                top_class = "COOLING_DEGRADATION_CYL2"
                conf = 91.0
                sev = min(1.0, res_cht2 / 6.0)
            elif res_map > 3.0:
                top_class = "TURBO_BOOST_DEFICIENCY"
                conf = 93.0
                sev = min(1.0, res_map / 6.0)
            elif res_vib > 3.0:
                top_class = "BEARING_WEAR_VIBRATION"
                conf = 90.0
                sev = min(1.0, res_vib / 6.0)
            else:
                top_class = "HEALTHY"
                conf = 99.8
                sev = 0.0

            all_probs = {c: (99.8 if c == top_class else 0.02) for c in FAULT_CLASSES}
            return top_class, conf, round(sev, 3), all_probs

        self.model.eval()
        x_vec = np.array([residual_dict.get(ch, 0.0) for ch in RESIDUAL_CHANNELS], dtype=np.float32).reshape(1, -1)
        with torch.no_grad():
            x = torch.tensor(x_vec).to(self.device)
            scaled_logits, severity_out = self.model(x)
            probs = torch.softmax(scaled_logits, dim=1).cpu().numpy()[0]
            severity = float(severity_out.item())

        top_idx = int(np.argmax(probs))
        top_class = IDX_TO_CLASS[top_idx]
        confidence_pct = round(float(probs[top_idx]) * 100.0, 1)

        all_probs = {IDX_TO_CLASS[i]: round(float(p) * 100.0, 2) for i, p in enumerate(probs)}
        return top_class, confidence_pct, round(severity, 3), all_probs

    def save(self, path: str):
        if HAS_TORCH and self.model is not None:
            torch.save(self.model.state_dict(), path)

    def load(self, path: str):
        if HAS_TORCH and self.model is not None:
            self.model.load_state_dict(torch.load(path, map_location=self.device))
            self.model.eval()
            self.is_trained = True
