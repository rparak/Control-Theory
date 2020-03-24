%% Exercise no. 6 (PART - 2) - INSTITUTE OF AUTOMATION AND COMPUTER SCIENCE
% Author: Ing. Roman Parák
% Subject: Control theory I
% Date: 31.10.2018

%% Zero-Pole example
clc
clear all
close all

s = tf('s');
%% System transfer
G_s = s*(3*s+1)/((2*s+1)*(s^2+2*s+5));

%% Test
pole(G_s)
zero(G_s)

%% Graph
pzmap(G_s)