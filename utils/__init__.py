from .data import download_and_load_data, extract_joint_values


def __getattr__(name):
    """Lazy import for visualization (matplotlib dependency)."""
    if name == "create_trajectory_gif":
        from .visualization import create_trajectory_gif
        return create_trajectory_gif
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


__all__ = ["create_trajectory_gif", "download_and_load_data", "extract_joint_values"]
