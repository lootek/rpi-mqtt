FROM python:3.12.0

RUN apt-get update && apt-get install -y \
  build-essential \
  python-dev-is-python3

COPY requirements.txt .
RUN pip install --user -r requirements.txt

COPY adafruit.patch .

RUN git clone https://github.com/adafruit/Adafruit_Python_DHT.git && \
  cd Adafruit_Python_DHT/ && \
  python setup.py install

#  patch Adafruit_DHT/platform_detect.py ../adafruit.patch && \

WORKDIR /code

COPY dht22.py .
CMD ["python", "-u", "/code/dht22.py"]
