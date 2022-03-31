function result = generate_uplink(M, N, in_f, out_f)
    Data = readmatrix(in_f);
    mimoChan = comm.MIMOChannel('SpatialCorrelationSpecification', 'None', 'NumTransmitAntennas', M, 'NumReceiveAntennas', N);
    output_data = mimoChan(Data);
    writematrix(output_data, out_f, 'delimiter', ' ');
    result = 0
end