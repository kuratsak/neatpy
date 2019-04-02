# neatpy
Simple interface for talking to most Neato robots via USB.

```import neato
xv = neato.xv11()
print xv.command('help')

# try this next to get a direct shell
xv.shell()
```