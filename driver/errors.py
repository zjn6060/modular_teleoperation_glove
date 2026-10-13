"""Exceptions raised by the motor driver."""


class DriverError(RuntimeError):
    """Base class for driver failures."""


class ConnectionError(DriverError):
    """The serial bus could not be opened or configured."""


class CommunicationError(DriverError):
    """A motor did not complete a bus operation successfully."""


class ConfigurationError(DriverError):
    """A motor or driver configuration is invalid."""


class SafetyError(DriverError):
    """An operation was rejected because a safety precondition was not met."""
