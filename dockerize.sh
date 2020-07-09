# # First, copy the necessary items into the docker/ folder
# cp -rv ./build/  ./docker/
# cp -rv ./test/   ./docker/
# cp -rv ./data/   ./docker/


# Then build the docker image
docker build -t boos:Dockerfile docker/

# Then run the docker image as a container
docker run -ti boos:Dockerfile
