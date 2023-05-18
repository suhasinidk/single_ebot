#!/bin/bash

function check-pkg-installed
{
	if [ $# -eq 0 ]; then
		echo "Please specify package name" 1>&2
		exit -1
	else
		if `apt list --installed 2>/dev/null | grep -iqwe ^$1/`; then
			if `apt list 2>/dev/null | grep -iqwe ^$1/`; then
				echo "Package $1 does not exist" 1>&2
				exit -1
			fi
			return 0
		else
			return 1
		fi
	fi
}

function install-pkg
{
	if [ $# -eq 0 ]; then
		echo "Please specify package name" 1>&2
		exit -1
	else
		if `apt list --installed 2>/dev/null | grep -iqwe ^$1/`; then
			if `apt list 2>/dev/null | grep -iqwe ^$1/`; then
				echo "Package $1 does not exist" 1>&2
				exit -1
			else
				sudo apt-get install $1
			fi
		else
			echo "Found package $1"
		fi
	fi
}

function install-pkgs
{
  for pkg in $*; do
    install-pkg $pkg
   done
}

function user-in-group
{
	if cut -d: -f1 /etc/group | grep -iqw $1; then
		if id -nG "$USER" | grep -iqvw $1; then
		    echo "$USER does not belong to group @1" 1>&2
		    echo "Adding $USER to group @1" 1>&2
		    sudo usermod -a -G docker $USER
		    echo "Reboot required!" 1>&2
		    exit -1
		fi
	else
		echo "Group $1 does not exist!" 1>&2
		exit -1
	fi
	echo "$USER already belongs to group $1"
}

function download-file
{
	curl $1
}

function load-docker-image
{
	docker load -i $1
}

function download-repo
{
	if [ -d $2 ]; then
		cd $2
		if [ -d .git ]; then
			git pull origin master
		else
			echo "Folder exists, but is not a repository!"
			exit -1
		fi
		cd - >/dev/null
	else
		git clone $1 $2
	fi
}

function install-repo-dependencies
{
  rosdep update
  sudo rosdep install --from-paths src --ignore-src --rosdistro=$1 -y
  find . -name "requirements.txt" | xargs -L1 python -m pip install --user -r
}

function build-repo
{
  catkin build -c --no-notify -s
}

function add-source-to-bash
{
	. $1
	if cat $HOME/.bashrc | grep -iqw $1; then
		echo "$SOURCE has already been added to bashrc"
	else
		echo "Adding source $1 to bashrc"
		echo ". $1" >> $HOME/.bashrc
	fi
}
