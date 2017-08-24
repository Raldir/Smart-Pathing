#pragma once

const float _CAR_SPEED = 10;
//IRRELEVANT const float _CAR_LENGTH = 10;
const int _CAR_MINIMUM_GAP = 10;

//TODO Bessere Methodik finden
const int _TIMETABLE_SPAN = 1000;

//TRAFFIC LIGHT
/*
#define _TRAFFIC_LIGHT_PERIOD
#define _TRAFFIC_LIGHT_DURATION
*/
const int TRAFFICLIGHT_DURATION = 10;
const int _SIMULATION_TICKS = 6000;
const int _SIMULATION_SECONDS_PER_TICK = 5;

const float _CAR_SPEED_PER_TICK = _SIMULATION_SECONDS_PER_TICK * _CAR_SPEED;

const int BASE_SPAWN_RATE = 10;

const int VERTEX_PRIORITY_DIVERGENCE = 5;

//TODO Ändere zu: Average_EDGELENGTH / VALUE
const int _CAR_RELEVANCE = 10;
