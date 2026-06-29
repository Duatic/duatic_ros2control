# FreezeController

This controller immediately puts any DuaDrive or other Duatic product into the state of a controlled safe stop.

## Usage

1. Create an entry in your controllers.yaml and add it to your launch file infrastructure.
You may activate this controller on startup
2. Activate the controller -> The arm should now be stiff
3. Deactivate the controller -> The arm is now movable again

All ros2control implementations of a Duatic product provide to ways to access the `freeze mode` interface of a specific actuator:
1. Via the actuator specific freeze mode interface. Example: `shoulder_flexion/freeze_mode` (Only use this if you need to freeze single acutators)
2. Via the system specific freeze mode interface. Example: `DynaarmSystem/freeze_mode`

### Configuration example

```
freeze_controller:
  ros__parameters:
    names:
      # Do not add /freeze_mode
      - DynaarmSystem
```

### Interaction

None, only activate, deactivate


## Parameters

| Name | Description | Default | Bounds |
| ---- | ----------- | ------  | ------ |
| `names` | List of freeze mode interfaces without the `/freeze_mode` suffix | `DynaarmSystem` | - |
| `disable_at_deactivate` | Disable freeze mode at controller deactivation | true | - |

## Notes

* The `freeze_mode` interface / controllers wins over any other controller. It is also the fallback in case of any invalid command interface combination
* On Duatic DuaDrives with brakes we recommend to __not__ activate this controller on startup. It is not needed but can introduce mechanical stress on the brakes.
