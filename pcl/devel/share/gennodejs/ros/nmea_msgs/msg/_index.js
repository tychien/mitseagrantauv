
"use strict";

let Gpgsa = require('./Gpgsa.js');
let Gpgga = require('./Gpgga.js');
let Sentence = require('./Sentence.js');
let Gprmc = require('./Gprmc.js');
let Gpgsv = require('./Gpgsv.js');
let GpgsvSatellite = require('./GpgsvSatellite.js');

module.exports = {
  Gpgsa: Gpgsa,
  Gpgga: Gpgga,
  Sentence: Sentence,
  Gprmc: Gprmc,
  Gpgsv: Gpgsv,
  GpgsvSatellite: GpgsvSatellite,
};
