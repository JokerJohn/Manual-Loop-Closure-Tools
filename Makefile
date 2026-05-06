PYTHON ?= python3
SESSION_ROOT ?=

.PHONY: help venv gtsam-python check test optimizer-help gui-help env-check backend docker-build docker-run-help assets clean

help:
	@echo "Targets:"
	@echo "  make venv       Create .venv and install requirements.txt"
	@echo "  make gtsam-python"
	@echo "                  Build/install the GTSAM 4.3 Python wrapper into the active venv"
	@echo "  make check      Run Python syntax and smoke checks"
	@echo "  make test       Run unit tests"
	@echo "  make optimizer-help"
	@echo "                  Print Python optimizer CLI help"
	@echo "  make gui-help   Print GUI help"
	@echo "  make env-check  Print local dependency summary"
	@echo "  make backend    Build the legacy C++ fallback optimizer"
	@echo "  make docker-build"
	@echo "                  Build the Docker image with the Python-first toolchain"
	@echo "  make docker-run-help"
	@echo "                  Print an example X11-enabled docker run command"
	@echo "  make assets SESSION_ROOT=/path/to/session"
	@echo "                  Capture README screenshots from a real session"
	@echo "  make clean      Remove common Python cache files"

venv:
	bash scripts/create_venv.sh

gtsam-python:
	bash scripts/install_gtsam_python.sh

check:
	$(PYTHON) -m py_compile launch_gui.py gui/manual_loop_closure_tool.py gui/manual_loop_closure/*.py gui/manual_loop_closure/python_optimizer/*.py gui/merge_pcds.py scripts/*.py
	$(PYTHON) launch_gui.py --help
	$(PYTHON) gui/manual_loop_closure/python_optimizer/cli.py --help
	$(PYTHON) scripts/check_env.py
	$(PYTHON) -m unittest discover -s tests

test:
	$(PYTHON) -m unittest discover -s tests

optimizer-help:
	$(PYTHON) gui/manual_loop_closure/python_optimizer/cli.py --help

gui-help:
	$(PYTHON) launch_gui.py --help

env-check:
	$(PYTHON) scripts/check_env.py

backend:
	bash scripts/build_backend_catkin.sh

docker-build:
	docker build -t manual-loop-closure-tools:latest .

docker-run-help:
	@echo "xhost +local:docker"
	@echo "docker run --rm -it \\"
	@echo "  --net=host \\"
	@echo "  -e DISPLAY=\$$DISPLAY \\"
	@echo "  -e QT_X11_NO_MITSHM=1 \\"
	@echo "  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \\"
	@echo "  -v /path/to/mapping_session:/data/session \\"
	@echo "  manual-loop-closure-tools:latest \\"
	@echo "  python launch_gui.py --session-root /data/session"

assets:
	@if [ -z "$(SESSION_ROOT)" ]; then echo "SESSION_ROOT is required"; exit 1; fi
	QT_QPA_PLATFORM=offscreen $(PYTHON) scripts/capture_readme_screenshots.py --session-root "$(SESSION_ROOT)"

clean:
	rm -rf __pycache__ gui/__pycache__ gui/manual_loop_closure/__pycache__ scripts/__pycache__
