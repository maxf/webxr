// fetch a timestamp every second and check if newer than now

let lastChange = parseInt(new Date().getTime());

const fetchTimestamp = () => {
  fetch('timestamp.txt?'+(new Date().getTime()))
    .then(response => response.text())
    .then(data => {
      setTimeout(fetchTimestamp, 2000);
      const ts = parseInt(data);
      if (ts > lastChange) {
        lastChange = ts;
        location.reload(true);
      }
    })
    .catch(error => console.log('Stopping because:', error));
};

fetchTimestamp();
