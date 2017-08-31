https://developers.google.com/optimization/

# Usage
    wget --output-document=/tmp/ortools_examples.tar.gz https://github.com/google/or-tools/releases/download/v6.3/or-tools_python_examples-ubuntu-14.04_v6.3.4431.tar.gz
    docker build . -t ortools
    docker run -it -v $(pwd):/data ortools bash