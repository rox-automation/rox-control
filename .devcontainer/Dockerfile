FROM python:latest

ARG USERNAME=dev
ARG UID=1000
ARG GID=1000
ARG VERSION="rox-control"
ARG DEBIAN_FRONTEND=noninteractive

# Create the user
RUN groupadd --gid $GID $USERNAME \
    && useradd --uid $UID --gid $GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# dialout group
RUN usermod -a -G dialout $USERNAME

# install packages
RUN apt-get install -y \
    can-utils \
    git \
    graphviz \
    locales \
    tree

# cleanup
RUN rm -rf /var/lib/apt/lists/*

# set locale
RUN export LC_ALL=en_US.UTF-8
RUN export LANG=en_US.UTF-8
RUN locale-gen en_US.UTF-8

# Install uv
RUN pip install --upgrade pip uv


USER ${USERNAME}
RUN echo 'export PS1="🐍  \[\033[1;36m\]'"${VERSION}"' \[\e[33m\]\W\[\e[m\] \[\033[1;36m\]# \[\033[0m\]"' >> ~/.bashrc

# add local bin to path
RUN echo "export PATH=\$PATH:/home/${USERNAME}/.local/bin" >> ~/.bashrc
ENV PATH="${PATH}:/home/${USERNAME}/.local/bin"

WORKDIR /home/${USERNAME}


# setup folders for saving vscode extensions
# https://code.visualstudio.com/remote/advancedcontainers/avoid-extension-reinstalls
RUN mkdir -p /home/$USERNAME/.vscode-server/extensions \
    && chown -R $USERNAME \
    /home/$USERNAME/.vscode-server


USER root

# workpace folder
RUN mkdir -p /workspaces && chown -R $USERNAME /workspaces

# build timestamp
RUN echo ${VERSION} >> /build_date.txt && \
    date >> /build_date.txt

USER ${USERNAME}
WORKDIR /home/${USERNAME}
