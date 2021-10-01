# How to Flash Xavier

For simplest installation, use 'Ubuntu 18.04' as the host computer and follow the instructions to [Download and Run SDK Manager](https://docs.nvidia.com/sdk-manager/download-run-sdkm/index.html) followed by [Installing Jetson Software with SDK Manager](https://docs.nvidia.com/sdk-manager/install-with-sdkm-jetson/index.html#install-with-sdkm-jetson).

If using 'Ubuntu 20', follow the instructions to use a the [SDK Manager Docker Image] (https://docs.nvidia.com/sdk-manager/docker-containers/index.html).


If the Docker instructions above fail when attempting the flashing in step, edit /usr/local/bin/sdkmanager to only consist of the following:
#!/bin/bash    

docker run -it --rm --privileged --net=host --volume /dev/bus/usb:/dev/bus/usb --volume /opt/sdkmanager:/home/nvidia --name sdkmanager sdkmanager-fixed $@

And then run the following command:
***sdkmanager --cli install --logintype devzone --product Jetson --version 4.6 --targetos Linux --target P2888-0001 --flash all --datacollection disable --license accept --exitonfinish***

Replace the version number 4.6 above with an updated version of the SDK Manager if that was available and downloaded when you started using these instructions.