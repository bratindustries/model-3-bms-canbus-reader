# model-3-bms-canbus-reader
reads cells voltages off the hv controller chargport canbus and vehicle canbus in a tesla model 3/y. 

based off work from evtv/collin kidder and Bryan Inkster
striped down the code to just read cell voltages, soc, temp

works with an ardunio due with a canbus interface


requires due_can libary https://github.com/collin80/due_can
