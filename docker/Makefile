build:
	docker build -t faster .

build-clone:
	docker build --build-arg DUMMY=`date +%s` -t faster .
run:
	docker run  -it \
			--gpus all \
			--env="NVIDIA_DRIVER_CAPABILITIES=all" \
			--privileged \
			-e DISPLAY=${DISPLAY} \
			--network=host \
			--env="DISPLAY" \
			--env="QT_X11_NO_MITSHM=1" \
			--volume=${PWD}/gurobi.lic:/opt/gurobi/gurobi.lic:ro \
			--rm faster