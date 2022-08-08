function rx_beacon_rssi = process_beacon(rx_beacon_cxdouble, tx_zero_prefix_len)
    total_users = size(rx_beacon_cxdouble, 2);
    total_frames = size(rx_beacon_cxdouble, 4);
    beacon_samp_start = tx_zero_prefix_len + 240; % 15 reps of STS (16-samps) at the start;
    beacon_samp_len = 256; % two reps of 128-samps Gold code;
    rx_beacon_rssi = zeros(total_users, total_frames);
    for u = 1:total_users
        rx_beacon = squeeze(rx_beacon_cxdouble(beacon_samp_start + 1:beacon_samp_start + beacon_samp_len, u, 1, :));
        rx_beacon_rssi(u, :) = 10 * log10(sum(abs(rx_beacon).^2, 1) / beacon_samp_len);
    end
end