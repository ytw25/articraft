# Errors

All public errors in the SDK derive from `sdk_hybrid.SDKError`.

## `SDKError`

```python
from sdk_hybrid import SDKError
```

Base exception for the articulated object SDK.

## `ValidationError`

```python
from sdk_hybrid import ValidationError
```

Raised when an articulated object definition or a QC check is invalid, inconsistent, or cannot be evaluated.
