/**
 * WebsocketChartComponent
 *
 * A custom element that connects to a websocket and displays the data in a chart.
 *
 * @extends HTMLElement
 *
 * @attribute {string} id - The id of the chart.
 * @attribute {string} port - The port of the websocket server.
 * @attribute {string} hostname - The hostname of the websocket server.
 */
class WebsocketChartComponent extends HTMLElement {
    /**
     * Constructor of the WebsocketChartComponent.
     * Calls the render method to create the component"s UI and establishes a websocket connection.
     */
    constructor() {
        super();

        this.attrs = {
            id: this.getAttribute("id"),
            port: this.getAttribute("port") || 8000,
            host: this.getAttribute("hostname") || "localhost",
        };

        this.render();
        this.connectWS();
        this.data = [[],[]]
    }

    /**
     * Establish a new websocket connection.
     * When a new message is received, update the chart.
     */
    connectWS() {
        this.ws = new WebSocket(`ws://${this.attrs.host}:${this.attrs.port}/ws/${this.attrs.id}`);
        this.ws.onerror = (event) => {
            console.error("Websocket error:", event);
        }
        this.ws.onmessage = (event) => {
            this.update_chart(event.data);
        }
    }

    /**
     * Update the chart with the new value.
     * @param val
     */
    update_chart(val) {
        this.data[0].push(this.data[0].length + 1);
        this.data[1].push(val);

        this.chart.setData(this.data);
    }

    /**
     * Render the component by creating a chart element.
     */
    render() {
        const header = document.createElement("h4")
        header.textContent = this.attrs.id;

        const chart_elem = document.createElement("div");

        this.appendChild(header);
        this.appendChild(chart_elem);

        const options = {
            width: 1200,
            height: 400,
            title: `Chart ${this.attrs.id}`,
            id: `chart-${this.attrs.id}`,
            series: [
                {},
                {
                    label: this.attrs.id,
                    stroke: "red",
                    width: 1/devicePixelRatio
                }
            ],
            axes: [
                {},
                {
                    label: this.attrs.id,
                }
            ],
            scales: {
                x: {
                    time: false,
                },
                y: {
                    auto: true,
                }
            }
        };

        this.chart = new uPlot(options, this.data, chart_elem);
    }
}

customElements.define("websocket-chart", WebsocketChartComponent);
