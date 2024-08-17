{ pkgs ? import <nixpkgs> {}}:
let
	fhs = pkgs.buildFHSUserEnv {
		name = "my-fhs-environment";

		targetPkgs = _: [ pkgs.python3 ];

		profile = ''
			set -e
			test -f .venv || python -m venv .venv --copies
			export PATH=.venv/bin:$PATH
			python -m pip install robotpy > /dev/null
			echo -e "\033[0;31mignore all that.\033[0m"
			test -f pyproject.toml || echo run this: python -m robotpy init\; sed 's/# "all"/  "all"/' -i pyproject.toml\; python -m robotpy sync
			set +e
		'';
	};
in fhs.env
