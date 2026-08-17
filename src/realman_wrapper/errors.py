class RealManWrapperError(RuntimeError):
    """Base exception for wrapper lifecycle and safety failures."""


class StaleSampleError(RealManWrapperError):
    """A state, frame, or command timestamp exceeded its configured age."""


class MotionAuthorizationError(RealManWrapperError):
    """A motion was attempted without a valid one-shot authorization."""


class SafetyViolation(RealManWrapperError):
    """A target or device state violated the configured safety envelope."""
