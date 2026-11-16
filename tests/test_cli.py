import pytest

from bipe.__main__ import main


def test_sim_help_reaches_the_sim_parser(capsys):
    with pytest.raises(SystemExit) as exit_info:
        main(["sim", "--help"])
    assert exit_info.value.code == 0
    assert "usage: bipe sim" in capsys.readouterr().out
