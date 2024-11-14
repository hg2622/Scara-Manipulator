clear all
close all
clc
vrclear
vrclose

addpath('../');
load('kinematic_traj.mat');
clik_phi_inverse;
sim('clik_phi_inverse.mdl', t);
%plot_outputs_inverse;
SCARA_VR_VISUALIZE(squeeze(q(:,1,:)), false);