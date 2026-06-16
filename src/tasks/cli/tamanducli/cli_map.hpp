#ifndef CLI_MAP_HPP
#define CLI_MAP_HPP

#include "wprotocol.hpp"

#include <cstring>

/**
 * @namespace cli
 * @brief Registro e despacho de comandos wire para handlers do firmware.
 *
 * Permite cadastrar novos comandos sem modificar o código da biblioteca.
 * Ver docs/specs.md.
 */
namespace cli {

/** @brief Número máximo de comandos registrados simultaneamente. */
constexpr int kMaxRegisteredCommands = 32;

/**
 * @brief Mapa de comandos com despacho para handlers e emissor wire integrado.
 *
 * @tparam MessageSize Tamanho máximo (bytes) de cada mensagem wire
 *                     (repassado a wire::Protocol).
 *
 * Fluxo típico:
 * @code
 * static void pushMessage(const char* msg) { bleEnqueue(msg); }
 *
 * static bool onHelp(const wire::Command& cmd, wire::WireView view,
 *                    wire::Protocol<256>& proto) {
 *   return proto.emitSingleResponse("help", {"ok"});
 * }
 *
 * cli::CliMap<256> cli(pushMessage);
 * cli.registerCommand("help", onHelp);
 * cli.processMessage("help(s,r);");
 * @endcode
 */
template <size_t MessageSize = 256> class CliMap {
public:
  /** @brief Emissor wire usado pelos handlers para responder. */
  using Protocol = wire::Protocol<MessageSize>;

  /**
   * @brief Assinatura de um handler de comando.
   *
   * @param cmd Comando parseado (modo, role, argumentos).
   * @param view Vista dos argumentos de payload (sem modo/role/índice).
   * @param proto Emissor para montar e enviar respostas via `pushMessage`.
   * @return `true` se o comando foi tratado com sucesso.
   */
  using HandlerFn = bool (*)(const wire::Command &cmd, wire::WireView view,
                             Protocol &proto);

  /** @brief Callback de envio repassado ao wire::Protocol interno. */
  using PushMessageFn = typename Protocol::PushMessageFn;

  /**
   * @brief Constrói o mapa com a função de envio Bluetooth.
   * @param pushMessage Função que enfileira cada mensagem wire para envio.
   */
  explicit CliMap(PushMessageFn pushMessage) : proto_(pushMessage) {}

  /**
   * @brief Acesso ao emissor wire (mutável).
   * @return Referência ao Protocol interno.
   */
  Protocol &protocol() { return proto_; }

  /**
   * @brief Acesso ao emissor wire (somente leitura).
   * @return Referência const ao Protocol interno.
   */
  const Protocol &protocol() const { return proto_; }

  /**
   * @brief Registra ou substitui um handler para `name`.
   *
   * A chave é normalizada para minúsculas (case-insensitive).
   *
   * @param name Nome do comando (ex.: `"param_list"`).
   * @param handler Função chamada quando o comando é recebido.
   * @return `false` se `name`/`handler` forem nulos, o nome for inválido
   *         ou o limite kMaxRegisteredCommands for atingido.
   */
  bool registerCommand(const char *name, HandlerFn handler) {
    if(name == nullptr || handler == nullptr ||
       count_ >= kMaxRegisteredCommands) {
      return false;
    }
    const std::string key = wire::commandKeyLower(name);
    if(key.empty() || key.size() >= wire::kNameCap) {
      return false;
    }
    for(int i = 0; i < count_; i++) {
      if(strcmp(entries_[i].name, key.c_str()) == 0) {
        entries_[i].handler = handler;
        return true;
      }
    }
    memcpy(entries_[count_].name, key.c_str(), key.size() + 1);
    entries_[count_].handler = handler;
    count_++;
    return true;
  }

  /**
   * @brief Remove um comando do mapa.
   * @param name Nome do comando (case-insensitive).
   * @return `false` se `name` for nulo ou o comando não estiver registrado.
   */
  bool unregisterCommand(const char *name) {
    if(name == nullptr) {
      return false;
    }
    const std::string key = wire::commandKeyLower(name);
    for(int i = 0; i < count_; i++) {
      if(strcmp(entries_[i].name, key.c_str()) != 0) {
        continue;
      }
      if(i < count_ - 1) {
        entries_[i] = entries_[count_ - 1];
      }
      count_--;
      return true;
    }
    return false;
  }

  /**
   * @brief Busca o handler registrado para `name`.
   * @param name Nome do comando (case-insensitive).
   * @return Ponteiro para o handler ou `nullptr` se não encontrado.
   */
  HandlerFn findHandler(const char *name) const {
    if(name == nullptr) {
      return nullptr;
    }
    const std::string key = wire::commandKeyLower(name);
    for(int i = 0; i < count_; i++) {
      if(strcmp(entries_[i].name, key.c_str()) == 0) {
        return entries_[i].handler;
      }
    }
    return nullptr;
  }

  /**
   * @brief Despacha um comando parseado para o handler correspondente.
   * @param cmd Comando wire já parseado.
   * @return `false` se não houver handler registrado para `cmd.name`.
   */
  bool dispatch(const wire::Command &cmd) {
    HandlerFn handler = findHandler(cmd.name);
    if(handler == nullptr) {
      return false;
    }
    wire::WireView view{cmd};
    return handler(cmd, view, proto_);
  }

  /**
   * @brief Faz parse e despacha todos os comandos de uma mensagem BLE.
   *
   * Aceita mensagens com vários comandos separados por `;`
   * (ex.: `"help(s,r);param_list(s,r);"`).
   *
   * @param message String wire recebida do host.
   * @return `false` se o parse falhar; `true` se ao menos um comando
   *         foi despachado com sucesso.
   */
  bool processMessage(const char *message) {
    std::vector<wire::Command> commands;
    if(!wire::parseMessage(message, commands)) {
      return false;
    }
    bool handled = false;
    for(const wire::Command &cmd : commands) {
      if(dispatch(cmd)) {
        handled = true;
      }
    }
    return handled;
  }

  /**
   * @brief Retorna quantos comandos estão registrados no mapa.
   * @return Valor entre 0 e kMaxRegisteredCommands.
   */
  int registeredCount() const { return count_; }

private:
  /** @brief Entrada interna do mapa nome → handler. */
  struct Entry {
    char      name[wire::kNameCap]; /**< Nome normalizado (minúsculas). */
    HandlerFn handler;              /**< Handler associado ao comando. */
  };

  Protocol proto_;                             /**< Emissor wire interno. */
  Entry    entries_[kMaxRegisteredCommands]{}; /**< Tabela de comandos. */
  int      count_ = 0;                         /**< Comandos registrados. */
};

} // namespace cli

#endif // CLI_MAP_HPP
