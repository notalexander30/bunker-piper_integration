import os
import pathlib


def test_abot_skill_exists_with_required_contract():
    default_path = (
        pathlib.Path(__file__).parents[2]
        / "openclaw"
        / "skills"
        / "piper-touch-marker"
        / "SKILL.md"
    )
    configured_path = os.environ.get("PIPER_TOUCH_MARKER_SKILL", str(default_path))
    skill = pathlib.Path(configured_path)
    assert skill.is_file(), f"OpenClaw skill not found: {skill}"
    text = skill.read_text(encoding="utf-8")
    assert text.startswith("---\n")
    assert "name: piper-touch-marker" in text
    assert "touch the marker" in text
    assert "/health" in text
    assert "/tools/piper/approach-marker" in text
    assert "/tools/piper/touch-marker" in text
    assert "/tools/piper/go-home" in text
    assert "/tools/piper/save-home" in text
    assert "return_home_after" in text
    assert "joint_state_available" in text
    assert "execution_allowed=false" in text or "execution_allowed: true" in text
    assert "contact_confirmed" in text
    assert "single_moveit_marker_touch" in text
    assert "Never generate arbitrary MoveIt" in text
