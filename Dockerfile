FROM ubuntu:14.04

RUN apt-get update && apt-get install -y \
    zlib1g-dev \
    build-essential \
    python-setuptools \
    g++

RUN apt-get install -y wget

ADD or-tools_python_examples-ubuntu-14.04_v6.3.4431.tar.gz /tmp
#RUN wget --output-document=/tmp/ortools_examples.tar.gz https://github.com/google/or-tools/releases/download/v6.3/or-tools_python_examples-ubuntu-14.04_v6.3.4431.tar.gz
#RUN tar zxvf /tmp/ortools_examples.tar.gz
RUN cd /tmp/ortools_examples && make install