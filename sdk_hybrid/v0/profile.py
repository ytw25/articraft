from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True, frozen=True)
class HybridSdkProfile:
    package_name: str = "sdk_hybrid"
    docs_subdir: str = "docs"
    authoring_mode: str = "hybrid"


DEFAULT_PROFILE = HybridSdkProfile()
