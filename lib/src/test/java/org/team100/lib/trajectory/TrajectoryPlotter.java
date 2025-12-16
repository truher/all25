package org.team100.lib.trajectory;

import java.awt.Dimension;
import java.awt.Frame;
import java.util.List;
import java.util.function.ToDoubleFunction;

import javax.swing.JDialog;
import javax.swing.WindowConstants;

import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.NumberAxis;
import org.jfree.chart.axis.NumberTickUnit;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.VectorRenderer;
import org.jfree.data.Range;
import org.jfree.data.xy.VectorSeries;
import org.jfree.data.xy.VectorSeriesCollection;
import org.team100.lib.trajectory.path.Path100;
import org.team100.lib.trajectory.path.spline.HolonomicSpline;
import org.team100.lib.trajectory.timing.TimedPose;

import edu.wpi.first.math.geometry.Pose2d;

public class TrajectoryPlotter {
    public static final boolean SHOW = false;

    private final TrajectoryToVectorSeries converter;
    private final SplineToVectorSeries splineConverter;
    private final PathToVectorSeries pathConverter;

    public TrajectoryPlotter(double arrowLength) {
        converter = new TrajectoryToVectorSeries(arrowLength);
        splineConverter = new SplineToVectorSeries(arrowLength);
        pathConverter = new PathToVectorSeries(arrowLength);
    }

    public void plot(String name, Trajectory100 t) {
        VectorSeries converted = convert(name, t);
        Range xrange = xRange(t);
        Range yrange = yRange(t);
        if (SHOW)
            actuallyPlot(name, xrange, yrange, 1, converted);
    }

    public VectorSeries convert(String name, Trajectory100 t) {
        return converter.convert(name, t);
    }

    public VectorSeries convert(String name, Path100 path) {
        return pathConverter.convert(name, path);
    }

    public void plot(String name, List<HolonomicSpline> s) {
        VectorSeries converted = convert(name, s);
        Range xrange = xRange(s);
        Range yrange = yRange(s);
        if (SHOW)
            actuallyPlot(name, xrange, yrange, 1, converted);
    }

    public VectorSeries convert(String name, List<HolonomicSpline> s) {
        return splineConverter.convert(name, s);
    }

    public void actuallyPlot(
            String name,
            Range xrange,
            Range yrange,
            double tick,
            VectorSeries... converted) {
        VectorSeriesCollection dataSet = new VectorSeriesCollection();
        for (VectorSeries c : converted) {
            dataSet.addSeries(c);
        }
        XYPlot plot = new XYPlot(
                dataSet,
                new NumberAxis("X"),
                new NumberAxis("Y"),
                new VectorRenderer());
        NumberAxis domain = (NumberAxis) plot.getDomainAxis();
        NumberAxis range = (NumberAxis) plot.getRangeAxis();
        domain.setRangeWithMargins(xrange);
        range.setRangeWithMargins(yrange);
        domain.setTickUnit(new NumberTickUnit(tick));
        range.setTickUnit(new NumberTickUnit(tick));

        ChartPanel panel = new ChartPanel(new JFreeChart(plot));
        // "true" means "modal" so wait for close.
        JDialog frame = new JDialog((Frame) null, "plot", true);
        frame.setContentPane(panel);

        frame.setPreferredSize(new Dimension(500, 500));
        frame.pack();
        frame.setVisible(true);
        frame.setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
    }

    public Range xRange(Trajectory100 t) {
        return range(t, (p) -> p.state().getPose().pose().getX());
    }

    public Range yRange(Trajectory100 t) {
        return range(t, (p) -> p.state().getPose().pose().getY());
    }

    public Range xRange(List<HolonomicSpline> s) {
        return range(s, (p) -> p.getX());
    }

    public Range yRange(List<HolonomicSpline> s) {
        return range(s, (p) -> p.getY());
    }

    public Range xRange(Path100 path) {
        return range(path, (p) -> p.getX());
    }

    public Range yRange(Path100 path) {
        return range(path, (p) -> p.getY());
    }

    /** Adds margin */
    Range range(Trajectory100 t, ToDoubleFunction<TimedPose> f) {
        double max = Double.NEGATIVE_INFINITY;
        double min = Double.POSITIVE_INFINITY;
        for (TimedPose p : t.getPoints()) {
            double x = f.applyAsDouble(p);
            if (x + 0.1 > max)
                max = x + 0.1;
            if (x - 0.1 < min)
                min = x - 0.1;
        }
        return new Range(min, max);
    }

    /** Adds margin */
    Range range(List<HolonomicSpline> splines, ToDoubleFunction<Pose2d> f) {
        double max = Double.NEGATIVE_INFINITY;
        double min = Double.POSITIVE_INFINITY;

        for (HolonomicSpline spline : splines) {
            for (double j = 0; j < 0.99; j += 0.1) {
                Pose2d p = spline.getPose2d(j);
                double x = f.applyAsDouble(p);
                if (x + 0.1 > max)
                    max = x + 0.1;
                if (x - 0.1 < min)
                    min = x - 0.1;
            }
        }
        return new Range(min, max);
    }

    /** Adds margin */
    Range range(Path100 t, ToDoubleFunction<Pose2d> f) {
        double max = Double.NEGATIVE_INFINITY;
        double min = Double.POSITIVE_INFINITY;
        for (int i = 0; i < t.length(); ++i) {
            Pose2d p = t.getPoint(i).getPose().pose();
            double x = f.applyAsDouble(p);
            if (x + 0.1 > max)
                max = x + 0.1;
            if (x - 0.1 < min)
                min = x - 0.1;
        }
        return new Range(min, max);
    }

    public static void plot(
            Trajectory100 t,
            double arrows,
            double ticks) {
        TrajectoryPlotter plotter = new TrajectoryPlotter(arrows);
        VectorSeries series = plotter.convert("trajectory", t);
        Range xrange = plotter.xRange(t);
        Range yrange = plotter.yRange(t);

        if (SHOW)
            plotter.actuallyPlot("compare", xrange, yrange, ticks, series);
    }

    public static void plot(
            List<HolonomicSpline> splines,
            double arrows,
            double ticks) {
        TrajectoryPlotter plotter = new TrajectoryPlotter(arrows);
        VectorSeries series = plotter.convert("before", splines);
        Range xrange = plotter.xRange(splines);
        Range yrange = plotter.yRange(splines);

        if (SHOW)
            plotter.actuallyPlot("compare", xrange, yrange, ticks, series);
    }

    public static void plot(
            Path100 path,
            double arrows,
            double ticks) {
        TrajectoryPlotter plotter = new TrajectoryPlotter(arrows);
        VectorSeries series = plotter.convert("path", path);
        Range xrange = plotter.xRange(path);
        Range yrange = plotter.yRange(path);

        if (SHOW)
            plotter.actuallyPlot("compare", xrange, yrange, ticks, series);
    }
}
