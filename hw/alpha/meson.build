alpha_ss = ss.source_set()
alpha_ss.add(when: 'CONFIG_CLIPPER', if_true: files('clipper.c', 'rmc.c', 'tigcontrol.c'))
alpha_ss.add(when: 'CONFIG_TSUNAMI', if_true: files('tsunami.c'))
alpha_ss.add(when: 'CONFIG_TIGBUS', if_true: files('tigbus.c'))
alpha_ss.add(when: 'CONFIG_AM29F016', if_true: files('am29f016.c'))
alpha_ss.add(when: 'CONFIG_LX164', if_true: files('lx164.c'))
alpha_ss.add(when: 'CONFIG_PYXIS', if_true: files('pyxis.c'))

hw_arch += {'alpha': alpha_ss}
