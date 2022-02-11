## Installation

The file VCS.nmconnection contains a NetworkManager Connection definition
for connecting automatically to the VCS 802.1X network.

sudo cp VCS.nmconnection /etc/NetworkManager/system-connections/
sudo chmod 0600 /etc/NetworkManager/system-connections/VCS.nmconnection
systemctl restart NetworkManager

Be advised there is one non-standard property of this Connection definition.
Every RaspPi where it is installed will use the same UUID for the Connection.
This should not be a problem because it will still be unique on every RaspPi.
