# 📦 Installing OpenCPN

## Ubuntu/Debian

### Option 1: APT (Easiest)

```bash
sudo apt update
sudo apt install opencpn
```

### Option 2: Official PPA (Latest Version)

```bash
# Add PPA
sudo add-apt-repository ppa:opencpn/opencpn
sudo apt update
sudo apt install opencpn
```

### Option 3: Flatpak

```bash
sudo apt install flatpak
flatpak install flathub org.opencpn.OpenCPN
flatpak run org.opencpn.OpenCPN
```

## Other Linux Distributions

### Fedora

```bash
sudo dnf install opencpn
```

### Arch Linux

```bash
sudo pacman -S opencpn
```

## macOS

```bash
# Using Homebrew
brew install --cask opencpn

# Or download from:
# https://opencpn.org/OpenCPN/info/downloads.html
```

## Windows

Download installer from:
https://opencpn.org/OpenCPN/info/downloads.html

## Verify Installation

```bash
opencpn --version
```

Or just launch:
```bash
opencpn
```

## First Launch

1. OpenCPN will ask for chart directory
2. You can skip this and add charts later
3. Go to Options → Charts → Chart Files → Add Directory

## Quick Test

After installing, verify it works:

```bash
opencpn &
```

You should see the OpenCPN window with a world map.

---

**Once installed, proceed to README.md for configuration!** 🗺️

