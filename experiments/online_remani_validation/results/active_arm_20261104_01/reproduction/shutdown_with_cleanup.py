"""Normal GNOME shutdown, preserving the user's prompt preference as a client."""
import json
import os
from pathlib import Path
from gi.repository import Gio, GLib

os.environ['DBUS_SESSION_BUS_ADDRESS'] = 'unix:path=/run/user/1000/bus'
receipt = Path(__file__).parents[1] / 'shutdown_receipt.json'
settings = Gio.Settings.new('org.gnome.SessionManager')
old = settings.get_user_value('logout-prompt')
bus = Gio.bus_get_sync(Gio.BusType.SESSION, None)
name = 'org.gnome.SessionManager'
manager_path = '/org/gnome/SessionManager'
client_interface = name + '.ClientPrivate'
loop = GLib.MainLoop()
attempt = dict(method='normal GNOME Shutdown with registered preference cleanup client',
               original_prompt_user_value=None if old is None else old.unpack(),
               prompt_restored=False, request_accepted=False, events=[])

def save():
    data = json.loads(receipt.read_text())
    data['normal_shutdown_cleanup_attempt'] = attempt
    data['request_accepted'] = attempt['request_accepted']
    receipt.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')
    with receipt.open('rb') as stream:
        os.fsync(stream.fileno())

def restore():
    if old is None:
        settings.reset('logout-prompt')
    else:
        settings.set_value('logout-prompt', old)
    Gio.Settings.sync()
    attempt['prompt_restored'] = settings.get_user_value('logout-prompt') == old

def call(path, interface, method, parameters=None):
    return bus.call_sync(name, path, interface, method, parameters, None,
                         Gio.DBusCallFlags.NONE, 5000, None)

client_path = call(manager_path, name, 'RegisterClient',
                   GLib.Variant('(ss)', ('org.remani.ActiveArmExperimentShutdown', ''))).unpack()[0]

def client_signal(connection, sender, path, interface, member, parameters):
    attempt['events'].append(member)
    if member == 'EndSession':
        # GNOME has already passed confirmation and logind authorization.
        restore()
        attempt['request_accepted'] = True
        save()
        call(client_path, client_interface, 'EndSessionResponse', GLib.Variant('(bs)', (True, '')))
    elif member == 'QueryEndSession':
        call(client_path, client_interface, 'EndSessionResponse', GLib.Variant('(bs)', (True, '')))
    elif member in ('CancelEndSession', 'Stop'):
        restore()
        if member == 'CancelEndSession':
            attempt['request_accepted'] = False
        save()
        loop.quit()

subscription = bus.signal_subscribe(name, client_interface, None, client_path,
                                    None, Gio.DBusSignalFlags.NONE, client_signal)

def completed(connection, result):
    try:
        connection.call_finish(result)
        attempt['dbus_shutdown_reply'] = 'success'
        attempt['request_accepted'] = True
    except Exception as exc:
        attempt['dbus_shutdown_reply'] = str(exc)
        restore()
        loop.quit()
    save()

def timeout():
    attempt['cleanup_timeout'] = True
    restore()
    save()
    loop.quit()
    return GLib.SOURCE_REMOVE

try:
    if not settings.set_boolean('logout-prompt', False):
        raise RuntimeError('The normal prompt setting is not writable')
    Gio.Settings.sync()
    save()
    GLib.timeout_add_seconds(35, timeout)
    bus.call(name, manager_path, name, 'Shutdown', None, None,
             Gio.DBusCallFlags.NONE, 30000, None, completed)
    loop.run()
finally:
    restore()
    save()
    try:
        call(manager_path, name, 'UnregisterClient', GLib.Variant('(o)', (client_path,)))
    except Exception:
        pass
    print(json.dumps(attempt, ensure_ascii=False), flush=True)
