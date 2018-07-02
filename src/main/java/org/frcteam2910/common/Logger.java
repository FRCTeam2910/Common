package org.frcteam2910.common;

import java.io.PrintStream;
import java.util.Date;
import java.util.UUID;

public class Logger {
	private static final UUID RUN_INSTANCE_UUID = UUID.randomUUID();

	private static final Logger ANONYMOUS_LOGGER = new Logger();

	private static final Object outputStreamLock = new Object();
	private static PrintStream outputStream = new PrintStream(System.out);
	private static final Object errorStreamLock = new Object();
	private static PrintStream errorStream = new PrintStream(System.err);

	private final String name;

	public Logger() {
		this((String) null);
	}

	public Logger(Class<?> clazz) {
		this(clazz.getName());
	}

	public Logger(String name) {
		this.name = name;
	}

	public static Logger getAnonymousLogger() {
		return ANONYMOUS_LOGGER;
	}

	public static void setOutputStream(PrintStream outputStream) {
		synchronized (outputStreamLock) {
			Logger.outputStream = outputStream;
		}
	}

	public static void setErrorStream(PrintStream errorStream) {
		synchronized (errorStreamLock) {
			Logger.errorStream = errorStream;
		}
	}

	private static void log(PrintStream out, Severity severity, String name, String msg) {
		if (name == null)
			out.format("<%s> (%s) [%s]: %s%n", RUN_INSTANCE_UUID, new Date(), severity, msg);
		else
			out.format("<%s> (%s) [%s/%s]: %s%n", RUN_INSTANCE_UUID, new Date(), name, severity, msg);
	}

	private void log(Severity severity, String msg) {
		switch (severity) {
			case DEBUG:
			case INFO:
				synchronized (outputStreamLock) {
					log(outputStream, severity, name, msg);
				}

				break;
			case WARNING:
			case ERROR:
				synchronized (errorStreamLock) {
					log(errorStream, severity, name, msg);
				}

				break;
			default:
				break;
		}
	}

	private void log(Severity severity, Throwable throwable) {
		switch (severity) {
			case DEBUG:
			case INFO:
				synchronized (outputStreamLock) {
					log(outputStream, severity, name, "An unhandled exception has occurred.");
					throwable.printStackTrace(outputStream);
				}

				break;
			case WARNING:
			case ERROR:
				synchronized (errorStreamLock) {
					log(errorStream, severity, name, "An unhandled exception has occurred.");
					throwable.printStackTrace(errorStream);
				}

				break;
			default:
				break;
		}
	}

	public void debug(String format, Object... args) {
		log(Severity.DEBUG, String.format(format, args));
	}

	public void debug(Throwable throwable) {
		log(Severity.DEBUG, throwable);
	}

	public void info(String format, Object... args) {
		log(Severity.INFO, String.format(format, args));
	}

	public void info(Throwable throwable) {
		log(Severity.INFO, throwable);
	}

	public void warn(String format, Object... args) {
		log(Severity.WARNING, String.format(format, args));
	}

	public void warn(Throwable throwable) {
		log(Severity.WARNING, throwable);
	}

	public void error(String format, Object... args) {
		log(Severity.ERROR, String.format(format, args));
	}

	public void error(Throwable throwable) {
		log(Severity.ERROR, throwable);
	}

	public enum Severity {
		DEBUG,
		INFO,
		WARNING,
		ERROR
	}
}
