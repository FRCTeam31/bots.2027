package frc.robot.subsystems.leds;

import org.prime.util.LEDPatternUtil;

import org.wpilib.units.Units;
import org.wpilib.units.measure.Time;
import org.wpilib.hardware.led.LEDPattern;
import org.wpilib.util.Color;

public class LEDPatterns {

        // ------------------------ Timing Constants --------------------------
        private static final Time SLOW_BLINK = Units.Milliseconds.of(500);
        private static final Time FAST_BLINK = Units.Milliseconds.of(100);
        private static final Time QUICK_FLASH_ON = Units.Milliseconds.of(50);
        private static final Time QUICK_FLASH_OFF = Units.Milliseconds.of(100);
        private static final Time SLOW_ALTERNATE = Units.Milliseconds.of(500);
        private static final Time FAST_ALTERNATE = Units.Milliseconds.of(100);

        // ------------------------ Red --------------------------------
        public static final LEDPattern RedSlowBlink = slowBlink(Color.RED);
        public static final LEDPattern RedFastBlink = fastBlink(Color.RED);
        public static final LEDPattern RedQuickFlash = quickFlash(Color.RED);
        public static final LEDPattern RedSlowBurstBlink = slowBurstBlink(Color.RED);
        public static final LEDPattern RedFastBurstBlink = fastBurstBlink(Color.RED);
        public static final LEDPattern RedAlternatingSlow = alternatingSlow(Color.RED);
        public static final LEDPattern RedAlternatingFast = alternatingFast(Color.RED);
        public static final LEDPattern RedTwoToneSlow = twoToneSlow(Color.RED);
        public static final LEDPattern RedTwoToneFast = twoToneFast(Color.RED);

        // ------------------------ Yellow -----------------------------
        public static final LEDPattern YellowSlowBlink = slowBlink(Color.YELLOW);
        public static final LEDPattern YellowFastBlink = fastBlink(Color.YELLOW);
        public static final LEDPattern YellowQuickFlash = quickFlash(Color.YELLOW);
        public static final LEDPattern YellowSlowBurstBlink = slowBurstBlink(Color.YELLOW);
        public static final LEDPattern YellowFastBurstBlink = fastBurstBlink(Color.YELLOW);
        public static final LEDPattern YellowAlternatingSlow = alternatingSlow(Color.YELLOW);
        public static final LEDPattern YellowAlternatingFast = alternatingFast(Color.YELLOW);
        public static final LEDPattern YellowTwoToneSlow = twoToneSlow(Color.YELLOW);
        public static final LEDPattern YellowTwoToneFast = twoToneFast(Color.YELLOW);

        // ------------------------ Blue -------------------------------
        public static final LEDPattern BlueSlowBlink = slowBlink(Color.BLUE);
        public static final LEDPattern BlueFastBlink = fastBlink(Color.BLUE);
        public static final LEDPattern BlueQuickFlash = quickFlash(Color.BLUE);
        public static final LEDPattern BlueSlowBurstBlink = slowBurstBlink(Color.BLUE);
        public static final LEDPattern BlueFastBurstBlink = fastBurstBlink(Color.BLUE);
        public static final LEDPattern BlueAlternatingSlow = alternatingSlow(Color.BLUE);
        public static final LEDPattern BlueAlternatingFast = alternatingFast(Color.BLUE);
        public static final LEDPattern BlueTwoToneSlow = twoToneSlow(Color.BLUE);
        public static final LEDPattern BlueTwoToneFast = twoToneFast(Color.BLUE);

        // ------------------------ Green ---------------------
        public static final LEDPattern GreenSlowBlink = slowBlink(Color.GREEN);
        public static final LEDPattern GreenFastBlink = fastBlink(Color.GREEN);
        public static final LEDPattern GreenQuickFlash = quickFlash(Color.GREEN);
        public static final LEDPattern GreenSlowBurstBlink = slowBurstBlink(Color.GREEN);
        public static final LEDPattern GreenFastBurstBlink = fastBurstBlink(Color.GREEN);
        public static final LEDPattern GreenAlternatingSlow = alternatingSlow(Color.GREEN);
        public static final LEDPattern GreenAlternatingFast = alternatingFast(Color.GREEN);
        public static final LEDPattern GreenTwoToneSlow = twoToneSlow(Color.GREEN);
        public static final LEDPattern GreenTwoToneFast = twoToneFast(Color.GREEN);

        // -------------------- Alternating Blink Patterns ----------------------
        public static final LEDPattern GreenRedAlternating = LEDPatternUtil.alternating(
                        GreenFastBlink,
                        RedFastBlink,
                        SLOW_ALTERNATE);
        public static final LEDPattern BlueYellowAlternating = LEDPatternUtil.alternating(
                        BlueFastBlink,
                        YellowFastBlink,
                        SLOW_ALTERNATE);
        public static final LEDPattern BlueGreenAlternatingBurstBlink = LEDPatternUtil.split(
                        BlueSlowBurstBlink,
                        GreenSlowBurstBlink,
                        LEDMap.PixelsPerSection / 2);

        // ==================== Factory Methods ================================

        /** Creates a slow-blinking pattern (500ms on, 500ms off) of the given color. */
        public static LEDPattern slowBlink(Color color) {
                return LEDPatternUtil.blink(color, SLOW_BLINK, SLOW_BLINK);
        }

        /** Creates a fast-blinking pattern (100ms on, 100ms off) of the given color. */
        public static LEDPattern fastBlink(Color color) {
                return LEDPatternUtil.blink(color, FAST_BLINK, FAST_BLINK);
        }

        /** Creates a fast-blinking pattern (100ms on, 100ms off) of the given color. */
        public static LEDPattern fastBlink(Color color, Time blinkTime) {
                return LEDPatternUtil.blink(color, blinkTime, blinkTime);
        }

        /** Creates a quick-flash pattern (50ms on, 100ms off) of the given color. */
        public static LEDPattern quickFlash(Color color) {
                return LEDPatternUtil.blink(color, QUICK_FLASH_ON, QUICK_FLASH_OFF);
        }

        /**
         * Creates a slow burst-blink pattern: a fast-blinking base shown for 500ms,
         * then off for 500ms, repeating.
         */
        public static LEDPattern slowBurstBlink(Color color) {
                return LEDPatternUtil.burstBlink(
                                fastBlink(color),
                                SLOW_BLINK,
                                SLOW_BLINK);
        }

        /**
         * Creates a fast burst-blink pattern: a fast-blinking base shown for 100ms,
         * then off for 100ms, repeating.
         */
        public static LEDPattern fastBurstBlink(Color color) {
                return LEDPatternUtil.burstBlink(
                                fastBlink(color),
                                FAST_BLINK,
                                FAST_BLINK);
        }

        /**
         * Creates a slow alternating pattern: one half shows the color while the
         * other is off, then they swap every 500ms.
         */
        public static LEDPattern alternatingSlow(Color color) {
                return LEDPatternUtil.alternating(
                                LEDPattern.solid(color),
                                LEDPattern.OFF,
                                SLOW_ALTERNATE);
        }

        /**
         * Creates a fast alternating pattern: one half shows the color while the
         * other is off, then they swap every 100ms.
         */
        public static LEDPattern alternatingFast(Color color) {
                return LEDPatternUtil.alternating(
                                LEDPattern.solid(color),
                                LEDPattern.OFF,
                                FAST_ALTERNATE);
        }

        /**
         * Creates a slow two-tone pattern: the given color alternates with black
         * across two halves of the strip, swapping every 500ms.
         */
        public static LEDPattern twoToneSlow(Color color) {
                return LEDPatternUtil.twoTone(color, Color.BLACK, SLOW_ALTERNATE);
        }

        /**
         * Creates a fast two-tone pattern: the given color alternates with black
         * across two halves of the strip, swapping every 100ms.
         */
        public static LEDPattern twoToneFast(Color color) {
                return LEDPatternUtil.twoTone(color, Color.BLACK, FAST_ALTERNATE);
        }

        /**
         * Creates a two-color alternating pattern: left half shows one color's
         * fast blink, right half shows the other, swapping sides every 500ms.
         */
        public static LEDPattern twoColorAlternating(Color left, Color right) {
                return LEDPatternUtil.alternating(
                                fastBlink(left),
                                fastBlink(right),
                                SLOW_ALTERNATE);
        }
}
