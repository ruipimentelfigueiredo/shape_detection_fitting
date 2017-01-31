#!/bin/bash

##### Functions

function get_data
{
	#sudo apt-get install unzip
	mkdir data
        cd data
	wget --no-check-certificate 'https://docs.google.com/uc?export=download&id=0B00mSh0AO3AHOVNtQkQzZDlVaWc' -O britney.mp4
	wget --no-check-certificate 'https://drive.google.com/open?id=0B00mSh0AO3AHSENQTndKTC1QMmc' -O wolf.mp4
	wget --no-check-certificate 'https://drive.google.com/open?id=0B00mSh0AO3AHbDNIOU1WOFllSWc' -O dengaz.mp4
}



# Get videos
get_data


