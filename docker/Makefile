build:
	docker build -t faster .

build-clone:
	docker build --build-arg DUMMY=`date +%s` -t faster .
run:
	docker run --volume=/home/jtorde/gurobi.lic:/opt/gurobi/gurobi.lic:ro -it --rm faster