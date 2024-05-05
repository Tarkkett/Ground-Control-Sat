$(document).ready(function () {
  const cttemp = document.getElementById("myTempChart").getContext("2d");
  const ctpressure = document.getElementById("myPressureChart").getContext("2d");

  const myTempChart = new Chart(cttemp, {
    type: "line",
    data: {
      datasets: [{ label: "Temperatūra C",  }],
    },
    options: {
      borderWidth: 3,
      borderColor: ['rgba(91, 201, 128, 1)',],
    },
  });
  const myPressureChart = new Chart(ctpressure, {
    type: "line",
    data: {
      datasets: [{ label: "Slėgis hPa",  }],
    },
    options: {
      borderWidth: 3,
      borderColor: ['rgba(255, 201, 128, 1)',],
    },
  });

  function addTempData(label, data) {
    myTempChart.data.labels.push(label);
    myTempChart.data.datasets.forEach((dataset) => {
      dataset.data.push(data);
    });
    myTempChart.update();
  }
  function addPressureData(label, data) {
    myPressureChart.data.labels.push(label);
    myPressureChart.data.datasets.forEach((dataset) => {
      dataset.data.push(data);
    });
    myPressureChart.update();
  }

  function removeFirstData() {
    myTempChart.data.labels.splice(0, 1);
    myTempChart.data.datasets.forEach((dataset) => {
      dataset.data.shift();
    });
  }

  const MAX_DATA_COUNT = 10;
  //connect to the socket server.
  //   var socket = io.connect("http://" + document.domain + ":" + location.port);
  var socket = io.connect();

  //receive details from server
  socket.on("updateTempSensorData", function (msg) {
    console.log("Received temperature sensorData :: " + msg.date + " :: " + msg.value);

    addTempData(msg.date, msg.value);
  });
  socket.on("updatePressureSensorData", function (msg) {
    console.log("Received presure sensorData :: " + msg.date + " :: " + msg.value);

    addPressureData(msg.date, msg.value);
  });
});
