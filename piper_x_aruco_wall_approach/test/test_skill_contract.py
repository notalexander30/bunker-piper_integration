import pathlib


def test_abot_skill_exists_with_required_contract():
    skill = pathlib.Path("/home/dase-hw101/ABot-Claw-piper-publish/openclaw_layer/skills/piper-touch-marker/SKILL.md")
    assert skill.is_file()
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
