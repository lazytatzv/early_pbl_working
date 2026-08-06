.PHONY: build dev edit monitor urdf clean simulate simulate-local simulate-global

build:
	uv run pio run

dev:
	uv run pio run -t upload

monitor:
	uv run pio device monitor

urdf:
	./update_urdf.sh $(URL)

edit:
	nvim src/main.cpp

clean:
	pio run -t clean
	rm -rf onshape_robot/robot.urdf onshape_robot/*.stl

simulate: simulate-local simulate-global simulate-pybullet

simulate-local:
	uv run python scripts/gait_simulation.py

simulate-global:
	uv run python scripts/gait_global_simulation.py

simulate-pybullet:
	docker compose up --build
