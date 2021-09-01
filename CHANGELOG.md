# CHANGELOG

## Version 0.2.0

* Use device containers to hold joints, fans, force sensors in Reachy and in each part.
* Add turn on/off methods in ReachySDK

### Bugfixes

* Wait for the sync loop to be ready before returning ReachySDK instance.
* Flush commands at exit.
* Fix async joint control issue.