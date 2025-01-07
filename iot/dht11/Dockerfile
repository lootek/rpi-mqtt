FROM python:latest

RUN apt-get update && apt-get install -y \
  build-essential \
  python-dev-is-python3

RUN git clone https://github.com/adafruit/Adafruit_Python_DHT.git && \
  cd Adafruit_Python_DHT/ && \
  python setup.py install

WORKDIR /code

COPY requirements.txt .
RUN pip install --user -r requirements.txt

COPY dht11.py .
CMD ["python", "-u", "/code/dht11.py"]
