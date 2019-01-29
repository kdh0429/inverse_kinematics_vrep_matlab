function rx_t = Cubic(rT, rT_0, rT_f, rx_0, rx_dot_0, rx_f, rx_dot_f)

if rT < rT_0
    
    rx_t = rx_0;
    
elseif rT>=rT_0 && rT<rT_f
    
    rx_t = rx_0 + rx_dot_0.*(rT-rT_0) + (3 .* (rx_f - rx_0)./ ((rT_f-rT_0) .* (rT_f-rT_0)) - 2 .* rx_dot_0./(rT_f-rT_0) - rx_dot_f./(rT_f-rT_0)).*(rT-rT_0).*(rT-rT_0)  + (-2 .* (rx_f - rx_0)./((rT_f-rT_0) .* (rT_f-rT_0) .* (rT_f-rT_0)) + (rx_dot_0 + rx_dot_f)./((rT_f-rT_0) .* (rT_f-rT_0))).*(rT-rT_0).*(rT-rT_0).*(rT-rT_0);
    
else
    rx_t = rx_f;
end
        
end