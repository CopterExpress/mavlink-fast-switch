[Unit]
Description=COEX fast MAVLink UDP switch for %I
After=network-online.target
Requires=network-online.target
PartOf=network-online.target
Documentation=https://github.com/CopterExpress/mavlink-fast-switch

[Service]
Type=simple
WorkingDirectory=/etc/mavlink-fast-switch
ExecStart=@CMAKE_INSTALL_PREFIX@/bin/mavlink-fast-switch /etc/mavlink-fast-switch/%i.conf 
RestartSec=2s
Restart=on-failure

[Install]
WantedBy=multi-user.target
