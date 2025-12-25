#!/usr/bin/env bash
set -e  

sudo add-apt-repository -y ppa:neovim-ppa/unstable
sudo apt-get update
sudo apt-get install -y neovim
sudo rm -rf /var/lib/apt/lists/*

sudo apt-get remove -y nodejs npm libnode-dev || true
sudo apt-get autoremove -y
sudo rm -rf /var/cache/apt/archives/nodejs_* /var/lib/apt/lists/*
curl -fsSL https://deb.nodesource.com/setup_current.x | sudo -E bash -
sudo apt-get install -y nodejs
sudo npm install -g npm@latest
node -v
npm -v
sudo rm -rf /var/lib/apt/lists/*

sudo apt-get update
sudo apt-get install -y clang clang-format python3-venv

mkdir -p ~/.config
git clone https://github.com/machinoro/neovim ~/.config/nvim

