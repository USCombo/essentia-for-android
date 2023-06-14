#!/bin/bash

# Abort if any command fails
set -e

# 1. Install the necessary dependencies
echo "Installing dependencies..."
sudo apt-get update
sudo apt-get install -y build-essential git cmake autoconf automake libtool pkg-config python3-pip python3-numpy unzip wget

# check if 'python' command is available
if ! command -v python &> /dev/null
then
    echo "Python could not be found. Creating a symbolic link to Python3..."
    sudo ln -s /usr/bin/python3 /usr/bin/python
fi

# 2. Download and extract Android NDK if it doesn't exist
if [ ! -d "android-ndk-r21d" ]; then
    echo "Downloading Android NDK..."
    wget https://dl.google.com/android/repository/android-ndk-r21d-linux-x86_64.zip
    unzip android-ndk-r21d-linux-x86_64.zip
fi

# 3. Set the NDK path variable
export ANDROID_NDK=$(pwd)/android-ndk-r21d

# 4. Clone the Essentia repository if it doesn't already exist in the current directory
if [ ! -d "essentia" ]; then
    echo "Cloning the Essentia repository..."
    git clone https://github.com/MTG/essentia.git
fi

# 5. Navigate to the Essentia directory
cd essentia

# 6. Set the Essentia environment variables
export ESSENTIA_BUILD_PACKAGING=ON
export ESSENTIA_BUILD_TESTS=OFF
export ESSENTIA_BUILD_EXAMPLES=OFF
export ESSENTIA_BUILD_PYTHON=OFF

# 7. Run the Waf configure command for Android
./waf configure --cross-compile-android --android-ndk=${ANDROID_NDK}

# 8. Run the Waf build command
./waf

echo "Build process completed!"

