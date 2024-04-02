class WebsocketTextComponent extends HTMLElement {

    constructor() {
        super();

        this.render();
        this.connectWS();
    }

    render() {
        // create an image tag
        const header = document.createElement('h4')
        header.textContent = this.getAttribute('id');

        this.text = document.createElement('div');
        this.text.classList.add('text_field');
        const button = document.createElement('button');
        button.textContent = 'Toggle';
        button.onclick = () => this.toggleWS();

        // append the image and button to the dom
        this.appendChild(header);
        this.appendChild(button);
        this.appendChild(this.text);
    }

    toggleWS() {
        // send a toggle message to the server
        console.log('toggle')
        this.ws.send('toggle');
    }

    connectWS() {
        // create a new websocket connection
        this.ws = new WebSocket(`ws://localhost:8000/ws/${this.getAttribute('id')}`);
        this.ws.onerror = (event) => {
            console.error('Websocket error:', event);
        }
        // when a new message is received, update the image. the image is send as base64 encoded jpeg
        this.ws.onmessage = (event) => {
            // convert to text
            console.log(event.data)
            const text_elem = document.createElement('p');
            text_elem.textContent = event.data;
            if(this.getAttribute('append') == 'true'){
                // prepend child
                this.text.prepend(text_elem);
            } else {
                this.text.innerHTML = '';
                this.text.appendChild(text_elem);
            }
        }
    }
}

// define the custom element
customElements.define('websocket-text', WebsocketTextComponent);