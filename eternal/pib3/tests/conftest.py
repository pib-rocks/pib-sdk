"""Shared pytest fixtures for pib3 tests."""

import pytest


@pytest.fixture(autouse=True)
def clear_module_caches():
    """Clear all module-level caches before each test.

    Prevents stale cached data from leaking between tests and causing
    order-dependent failures.
    """
    # Clear joint limits cache
    from pib3.backends.base import clear_joint_limits_cache
    clear_joint_limits_cache()

    yield

    # Clear again after test to avoid polluting the next one
    clear_joint_limits_cache()

    # Clear DH model caches (only if dh_model was imported)
    try:
        from pib3.dh_model import clear_caches
        clear_caches()
    except ImportError:
        pass
