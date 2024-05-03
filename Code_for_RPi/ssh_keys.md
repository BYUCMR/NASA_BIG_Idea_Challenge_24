# Setting up SSH keys to provide push/pull permissions to Github Repositories
## Purpose
This document provides some links for configuring [SSH Keys](https://docs.github.com/en/authentication/connecting-to-github-with-ssh) on a RPi to allow for push/pull permissions to Github repositories. This is useful for users who want to contribute to a repository without having to enter their username and password each time they push or pull changes.

## Table of Contents
- [Creating SSH Keys](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent)
- [Adding SSH Keys to Github](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account)
- [Connecting to NASA Project Repository](#nasa-project-repository)
- [Setting Authorship in Git](#git-authorship)
- [Setting your Branch](#git-branches)

## NASA Project Repository
After your SSH keys are set up, open a terminal, navigate to the desired directory to clone the repo (such as `Documents`) and run the following commands in a terminal:
```bash
git clone "SSH URL of the repository" (no quotes)
```

## Git Authorship
Before you push changes to a repository, you must set your authorship. This is done by running the following commands in a terminal:
```bash
git config --global user.name "Your Name"
git config --global user.email "Your github email"
```
This will set your name and email address and tag you as the author of any changes you make to the repository. This is important for tracking changes. These parameters will be static on a personal machine, but should be reconfigured on the RPi every time a different individual uses it to push changes to the repository.

## Git Branches
When you clone a repository, you will be on the `main` branch by default. It is generally bad practice to push changes directly to the `main` branch. Instead, you should create a new branch for your changes. This is done by running the following commands in a terminal:
```bash
git checkout -b "branch_name"
```
This will create a new branch and switch to it. You can then make changes to the code, commit them, and push them to the repository. When you are ready to merge your changes into the `main` branch, you can create a pull request on the Github website. The basics of Git branching and merging can be found [here](https://git-scm.com/book/en/v2/Git-Branching-Basic-Branching-and-Merging)