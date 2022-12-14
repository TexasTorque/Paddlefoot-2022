/**
 * Copyright 2022 Texas Torque.
 * 
 * This file is part of Paddlefoot-2022, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@gtsbr.org>.
 */
package org.texastorque;

public final class Ports {
    public static final class DRIVEBASE {
        public static final class TRANSLATIONAL {
            public static final class RIGHT {
                public static final int FRONT = 2;
                public static final int BACK = 4;
            }
            public static final class LEFT {
                public static final int FRONT = 1;
                public static final int BACK = 3;
            }
        }
        public static final class ROTATIONAL {
            public static final class RIGHT {
                public static final int FRONT = 6;
                public static final int BACK = 8;
            }
            public static final class LEFT {
                public static final int FRONT = 5;
                public static final int BACK = 7;
            }
        }
    }

    public static final class SHOOTER {
        public static final int HOOD = 16;
        public static final class FLYWHEEL {
            public static final int LEFT = 14;
            public static final int RIGHT = 15;
        }
    }

    public static final class CLIMBER {
        public static final class LIFT {
            public static final int LEFT = 9;
            public static final int RIGHT = 10;
        }
        public final static int HATCH = 11;
    }

    public static final int TURRET = 18;

    public static final int LIGHTS = 3;
}
