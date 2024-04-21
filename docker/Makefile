build:
	docker build -f /home/jtorde/Research/faster_ws/src/faster/docker/Dockerfile -t faster .

run:
	docker run --volume=/home/jtorde/gurobi.lic:/opt/gurobi/gurobi.lic:ro -it \
				--privileged \
				--gpus all --env="NVIDIA_DRIVER_CAPABILITIES=all" \
				-e DISPLAY=$DISPLAY \
				--net=host --env="DISPLAY" \
				--env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
				--rm faster 