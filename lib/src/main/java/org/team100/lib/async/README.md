# async

Sometimes we want to do something periodically, but it doesn't need to be as frequent
as the 50 Hz robot main loop.  For example, we might want to check the dashboard for
changes in configuration widgets every second or two.  We use this for things like
setting the log level and scanning for new controllers.

The `lib.async` package contains a few different ways to do it, and a factory that
makes it easy to experiment with them.