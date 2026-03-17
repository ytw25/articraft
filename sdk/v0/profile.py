from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True, frozen=True)
class SdkProfile:
    package_name: str = "sdk"
    docs_subdir: str = "docs"
    authoring_mode: str = "base"


DEFAULT_PROFILE = SdkProfile()
