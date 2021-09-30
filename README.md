# METR4202 Ximea Tutorial
## Step 1: Installing Ximea Software Package
- If you're on Ubuntu (Linux), make sure all of your packages are up to date, and that you have these installed
```
sudo apt-get update && sudo apt-get install build-essential linux-headers-"$(uname -r)" 
```
- On any machine running *Ubuntu*, download this software package and follow instructions from the ximea website [here](https://www.ximea.com/support/wiki/apis/ximea_linux_software_package).
- Or paste this commands into the terminal
```
wget https://www.ximea.com/downloads/recent/XIMEA_Linux_SP.tgz
```
```
tar xzf XIMEA_Linux_SP.tgz
```
```
cd package
```
```
./install
```
You need to run the following command after each boot to disable the USB memory limits
```
echo 0 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
```
