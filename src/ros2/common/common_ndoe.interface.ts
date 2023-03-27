export interface Subscriber {
    start(): void;
    stop(): void;
};

export interface Publisher {
    start(): void;
    stop(): void;
};