$(document).ready(function () {
  const cttemp = document.getElementById("myTempChart").getContext("2d");
  const ctpressure = document.getElementById("myPressureChart").getContext("2d");
  const cthumid = document.getElementById("myHumidityChart").getContext("2d");
  const ctuva = document.getElementById("myUVAChart").getContext("2d");
  const ctuvb = document.getElementById("myUVBChart").getContext("2d");
  const ctuvc = document.getElementById("myUVCChart").getContext("2d");
  const ctairqual = document.getElementById("myAirQualChart").getContext("2d");
  const ctservox = document.getElementById("myServoXChart").getContext("2d");
  const ctservoy = document.getElementById("myServoYChart").getContext("2d");

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
  const myHumidityChart = new Chart(cthumid, {
    type: "line",
    data: {
      datasets: [{ label: "Drėgnumas %",  }],
    },
    options: {
      borderWidth: 3,
      borderColor: ['rgba(46, 67, 120, 1)',],
    },
  });
  const myUVAChart = new Chart(ctuva, {
    type: "line",
    data: {
      datasets: [{ label: "UVA",  }],
    },
    options: {
      borderWidth: 3,
      borderColor: ['rgba(87, 42, 120, 1)',],
    },
  });
  const myUVBChart = new Chart(ctuvb, {
    type: "line",
    data: {
      datasets: [{ label: "UVB",  }],
    },
    options: {
      borderWidth: 3,
      borderColor: ['rgba(122, 23, 94, 1)',],
    },
  });
  const myUVCChart = new Chart(ctuvc, {
    type: "line",
    data: {
      datasets: [{ label: "UVC",  }],
    },
    options: {
      borderWidth: 3,
      borderColor: ['rgba(161, 22, 59, 1)',],
    },
  });
  const myAirQualChart = new Chart(ctairqual, {
    type: "line",
    data: {
      datasets: [{ label: "Oro kokybė",  }],
    },
    options: {
      borderWidth: 3,
      borderColor: ['rgba(186, 227, 23, 1)',],
    },
  });
  const myServoXChart = new Chart(ctservox, {
    type: "line",
    data: {
      datasets: [{ label: "X",  }],
    },
    options: {
      borderWidth: 3,
      borderColor: ['rgba(10, 38, 8, 1)',],
    },
  });
  const myServoYChart = new Chart(ctservoy, {
    type: "line",
    data: {
      datasets: [{ label: "Y",  }],
    },
    options: {
      borderWidth: 3,
      borderColor: ['rgba(209, 68, 29, 1)',],
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
  function addHumidityData(label, data) {
    myHumidityChart.data.labels.push(label);
    myHumidityChart.data.datasets.forEach((dataset) => {
      dataset.data.push(data);
    });
    myHumidityChart.update();
  }
  function addUVAData(label, data) {
    myUVAChart.data.labels.push(label);
    myUVAChart.data.datasets.forEach((dataset) => {
      dataset.data.push(data);
    });
    myUVAChart.update();
  }
  function addUVBData(label, data) {
    myUVBChart.data.labels.push(label);
    myUVBChart.data.datasets.forEach((dataset) => {
      dataset.data.push(data);
    });
    myUVBChart.update();
  }
  function addUVCData(label, data) {
    myUVCChart.data.labels.push(label);
    myUVCChart.data.datasets.forEach((dataset) => {
      dataset.data.push(data);
    });
    myUVCChart.update();
  }
  function addAirQualData(label, data) {
    myAirQualChart.data.labels.push(label);
    myAirQualChart.data.datasets.forEach((dataset) => {
      dataset.data.push(data);
    });
    myAirQualChart.update();
  }
  function addServoXData(label, data) {
    myServoXChart.data.labels.push(label);
    myServoXChart.data.datasets.forEach((dataset) => {
      dataset.data.push(data);
    });
    myServoXChart.update();
  }
  function addServoYData(label, data) {
    myServoYChart.data.labels.push(label);
    myServoYChart.data.datasets.forEach((dataset) => {
      dataset.data.push(data);
    });
    myServoYChart.update();
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
  socket.on("updateHumiditySensorData", function (msg) {
    console.log("Received presure sensorData :: " + msg.date + " :: " + msg.value);

    addHumidityData(msg.date, msg.value);
  });
  socket.on("updateUVASensorData", function (msg) {
    console.log("Received presure sensorData :: " + msg.date + " :: " + msg.value);

    addUVAData(msg.date, msg.value);
  });
  socket.on("updateUVBSensorData", function (msg) {
    console.log("Received presure sensorData :: " + msg.date + " :: " + msg.value);

    addUVBData(msg.date, msg.value);
  });
  socket.on("updateUVCSensorData", function (msg) {
    console.log("Received presure sensorData :: " + msg.date + " :: " + msg.value);

    addUVCData(msg.date, msg.value);
  });
  socket.on("updateAirQualSensorData", function (msg) {
    console.log("Received presure sensorData :: " + msg.date + " :: " + msg.value);

    addAirQualData(msg.date, msg.value);
  });
  socket.on("updateServoXSensorData", function (msg) {
    console.log("Received presure sensorData :: " + msg.date + " :: " + msg.value);

    addServoXData(msg.date, msg.value);
  });
  socket.on("updateServoYSensorData", function (msg) {
    console.log("Received presure sensorData :: " + msg.date + " :: " + msg.value);

    addServoYData(msg.date, msg.value);
  });
});
