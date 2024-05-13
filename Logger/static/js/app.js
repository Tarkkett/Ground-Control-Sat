$(document).ready(function () {
  const cttemp = document.getElementById("myTempChart").getContext("2d");
  const ctpressure = document.getElementById("myPressureChart").getContext("2d");
  const cthumid = document.getElementById("myHumidityChart").getContext("2d");
  const ctverticalvel = document.getElementById("myVerticalVelChart").getContext("2d");
  const ctuvb = document.getElementById("myUVBChart").getContext("2d");
  const ctgrav = document.getElementById("myGravChart").getContext("2d");
  const ctalt = document.getElementById("myAltitudeChart").getContext("2d");
  const ctservo = document.getElementById("myServoChart").getContext("2d");

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
  const myServoChart = new Chart(ctservo, {
    type: "line",
    data: {
      datasets: [{ label: "Servo X",  }, {label: "Servo Y", }],
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
  const myGravChart = new Chart(ctgrav, {
    type: "line",
    data: {
      datasets: [{ label: "Gs pagreitis",  }],
    },
    options: {
      borderWidth: 3,
      borderColor: ['rgba(161, 22, 59, 1)',],
    },
  });
  const myAltitudeChart = new Chart(ctalt, {
    type: "line",
    data: {
      datasets: [{ label: "Aukštis",  }],
    },
    options: {
      borderWidth: 3,
      borderColor: ['rgba(186, 227, 23, 1)',],
    },
  });
  const myVerticalVelChart = new Chart(ctverticalvel, {
    type: "line",
    data: {
      datasets: [{ label: "Vertical Vel",  }],
    },
    options: {
      borderWidth: 3,
      borderColor: ['rgba(10, 38, 8, 1)',],
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
  function addVerticalVelData(label, data) {
    myVerticalVelChart.data.labels.push(label);
    myVerticalVelChart.data.datasets.forEach((dataset) => {
      dataset.data.push(data);
    });
    myVerticalVelChart.update();
  }
  function addUVBData(label, data) {
    myUVBChart.data.labels.push(label);
    myUVBChart.data.datasets.forEach((dataset) => {
      dataset.data.push(data);
    });
    myUVBChart.update();
  }
  function addAltitudeData(label, data) {
    myAltitudeChart.data.labels.push(label);
    myAltitudeChart.data.datasets.forEach((dataset) => {
      dataset.data.push(data);
    });
    myAltitudeChart.update();
  }
  function addGravData(label, data) {
    myGravChart.data.labels.push(label);
    myGravChart.data.datasets.forEach((dataset) => {
      dataset.data.push(data);
    });
    myGravChart.update();
  }
  function addServoData(label, datax, datay) {
    myServoChart.data.labels.push(label);
    myServoChart.data.datasets[0].data.push(datax)
    myServoChart.data.datasets[1].data.push(datay)
    myServoChart.update();
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
  socket.on("updateServoSensorData", function (msg) {
    addServoData(msg.date, msg.valuex, msg.valuey);
  });
  socket.on("updateUVBSensorData", function (msg) {
    console.log("Received presure sensorData :: " + msg.date + " :: " + msg.value);

    addUVBData(msg.date, msg.value);
  });
  socket.on("updateAltitudeSensorData", function (msg) {
    console.log("Received presure sensorData :: " + msg.date + " :: " + msg.value);

    addAltitudeData(msg.date, msg.value);
  });
  socket.on("updateGravSensorData", function (msg) {
    console.log("Received presure sensorData :: " + msg.date + " :: " + msg.value);

    addGravData(msg.date, msg.value);
  });
  socket.on("updateVerticalVelSensorData", function (msg) {
    console.log("Received presure sensorData :: " + msg.date + " :: " + msg.value);

    addVerticalVelData(msg.date, msg.value);
  });
});
