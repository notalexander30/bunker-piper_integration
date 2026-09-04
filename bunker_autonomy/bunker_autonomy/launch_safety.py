#!/usr/bin/env python3


def resolve_output_cmd_vel_topic(mode: str, override: str = '') -> str:
    """Resolve the final velocity topic while making dry-run isolation explicit."""
    normalized_mode = str(mode).strip().lower()
    normalized_override = str(override).strip()

    if normalized_mode not in ('dry_run', 'drive'):
        raise RuntimeError(
            "mode must be 'dry_run' or 'drive', got '%s'" % normalized_mode
        )

    if normalized_mode == 'dry_run':
        if normalized_override not in ('', '/cmd_vel_debug'):
            raise RuntimeError(
                "dry_run requires output_cmd_vel_topic='/cmd_vel_debug', got '%s'"
                % normalized_override
            )
        return '/cmd_vel_debug'

    return normalized_override or '/cmd_vel'
